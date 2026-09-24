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
The real arm's bring-up, end to end, with the PCA9685 simulated.

arm.launch.py is started exactly as on the Raspberry Pi, with dry_run:=true: the
description is expanded with the servo calibration, ros2_control_node loads the
driver as a plugin, the simulation's controllers come up on it, and a trajectory
is sent the way the draw node sends one. Everything except the I2C bus is the
code that will run on the arm.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f'joint_{i}' for i in range(1, 6)]
# Half a PCA9685 count at 50 Hz, as joint angle under the declared calibration:
# the most the driver's rounding can move a joint from its command.
HALF_COUNT = 0.5 * 4.88 / 636.62


@pytest.mark.launch_test
def generate_test_description():
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tattotronix_hardware'), 'launch', 'arm.launch.py')),
        launch_arguments={'dry_run': 'true'}.items(),
    )
    return launch.LaunchDescription([arm, launch_testing.actions.ReadyToTest()])


class TestDryRun(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_arm_dry_run')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _active_controllers(self, timeout=90.0):
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                if future.result() is not None:
                    active = {c.name for c in future.result().controller if c.state == 'active'}
                    if {'joint_state_broadcaster', 'arm_controller'} <= active:
                        return active
            time.sleep(0.5)
        self.fail('the controllers never became active on the PCA9685 driver')

    def test_the_arm_follows_a_trajectory_with_no_board_attached(self):
        self._active_controllers()
        latest = {}
        self.node.create_subscription(
            JointState, '/joint_states',
            lambda msg: latest.update(zip(msg.name, msg.position)), 10)

        client = ActionClient(
            self.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))
        target = [0.2, -0.1, 0.15, 0.0, 0.1]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=1))]
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=5.0)
        self.assertTrue(sent.result().accepted)
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=20.0)
        self.assertEqual(
            result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        # What /joint_states reports is what the driver sent, after rounding to
        # the board's step: the target to within half a count.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for name, want in zip(JOINTS, target):
            self.assertAlmostEqual(latest[name], want, delta=HALF_COUNT, msg=name)
