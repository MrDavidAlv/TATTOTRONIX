# Copyright 2026 Mario David Alvarez Vallejo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
Drive draw_strokes end to end, the way the tablet app drives it.

app.launch.py is started with the real arm's driver in a dry run.
Everything but the I2C bus is the code that runs on the Raspberry Pi: the
description, the PCA9685 driver as a plugin, the controllers, rosbridge and the
draw server. A square is drawn, its ink shows on /ink_trace on the panel, a
stroke off the panel is refused with the reason, and a cancel stops a drawing
and lifts the needle clear of the work.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tattotronix_control import arm, plan
from tattotronix_interfaces.action import DrawStrokes
from tattotronix_interfaces.msg import Stroke
from visualization_msgs.msg import Marker


@pytest.mark.launch_test
def generate_test_description():
    app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tattotronix_bringup'), 'launch', 'app.launch.py')),
        launch_arguments={'backend': 'arm', 'dry_run': 'true'}.items(),
    )
    return launch.LaunchDescription([app, launch_testing.actions.ReadyToTest()])


def square(x0, y0, side):
    return Stroke(x_mm=[x0, x0 + side, x0 + side, x0, x0], y_mm=[y0, y0, y0 + side, y0 + side, y0])


class TestDrawServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_draw_server')
        cls.marks = []
        cls.node.create_subscription(Marker, '/ink_trace', cls.marks.append, 10)
        cls.joints = {}
        cls.node.create_subscription(
            JointState, '/joint_states', lambda m: cls.joints.update(zip(m.name, m.position)), 10)
        cls.description = []
        cls.node.create_subscription(
            String, '/robot_description', lambda m: cls.description.append(m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.client = ActionClient(cls.node, DrawStrokes, '/draw_strokes')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _ready(self, timeout=90.0):
        lister = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if lister.wait_for_service(timeout_sec=1.0):
                future = lister.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                if future.result() is not None and {'arm_controller'} <= {
                        c.name for c in future.result().controller if c.state == 'active'}:
                    break
            time.sleep(0.5)
        else:
            self.fail('the arm controller never became active')
        self.assertTrue(self.client.wait_for_server(timeout_sec=30.0), 'no /draw_strokes')

    def _send(self, strokes, speed, on_feedback=None):
        goal = DrawStrokes.Goal(strokes=strokes, speed=speed)
        sent = self.client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        return sent.result()

    def _result(self, handle, timeout=60.0):
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), 'no result in time')
        return future.result().result

    def test_1_a_square_is_drawn_and_its_ink_is_on_the_panel(self):
        self._ready()
        seen = []
        handle = self._send([square(90.0, 60.0, 10.0)], 3.0, lambda f: seen.append(f.feedback))
        self.assertTrue(handle.accepted)
        result = self._result(handle)
        self.assertTrue(result.success, result.message)
        self.assertIn('drew 1 stroke,', result.message)
        self.assertGreater(result.seconds, 0.0)
        stages = {f.stage for f in seen}
        self.assertTrue({'planning', 'drawing'} <= stages, stages)
        self.assertGreater(max(f.progress for f in seen), 0.5)

        # The last trace holds the whole square: inside the panel, at the depth.
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        ink = [m for m in self.marks if m.ns == 'drawing' and m.points]
        self.assertTrue(ink, 'no ink on /ink_trace')
        pts = ink[-1].points
        xs = [p.x for p in pts]
        ys = [p.y for p in pts]
        # Panel millimetres (90..100, 60..70) are x 0.20..0.21 m, y -0.01..0 m in the arm frame.
        self.assertAlmostEqual(min(xs), 0.200, delta=0.0005)
        self.assertAlmostEqual(max(xs), 0.210, delta=0.0005)
        self.assertAlmostEqual(min(ys), -0.010, delta=0.0005)
        self.assertAlmostEqual(max(ys), 0.000, delta=0.0005)
        self.assertTrue(all(abs(p.z - 0.0035) < 1e-4 for p in pts))

    def test_2_a_stroke_off_the_panel_is_refused_with_the_reason(self):
        self._ready()
        handle = self._send([Stroke(x_mm=[10.0, 250.0], y_mm=[10.0, 10.0])], 1.0)
        self.assertTrue(handle.accepted)
        result = self._result(handle, timeout=20.0)
        self.assertFalse(result.success)
        self.assertIn('off the 200 x 140 mm panel', result.message)

    def test_3_a_cancel_stops_the_drawing_and_lifts_the_needle(self):
        self._ready()
        progress = []
        handle = self._send([square(20.0, 20.0, 60.0)], 1.0, lambda f: progress.append(f.feedback))
        self.assertTrue(handle.accepted)
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=10.0)
        result = self._result(handle, timeout=20.0)
        self.assertFalse(result.success)
        self.assertEqual(result.message, 'cancelled; the needle lifted clear of the work')
        self.assertLess(max(f.progress for f in progress if f.stage == 'drawing'), 1.0)
        self.assertIn('lifting', {f.stage for f in progress})

        # Where the arm is now, by the URDF it publishes: the tip at the travel
        # height, to within what the servos' step leaves of it.
        end = time.monotonic() + 2.0
        while time.monotonic() < end or not self.description:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        chain = arm.Chain(self.description[0])
        tip = chain.tcp([self.joints[name] for name in chain.names])
        self.assertAlmostEqual(tip[2], plan.PANEL_Z + plan.CLEARANCE_MM / 1000, delta=0.0015)
