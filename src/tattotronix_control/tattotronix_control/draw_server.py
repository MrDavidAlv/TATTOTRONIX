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
Draw the strokes a client sends: the draw_strokes action.

The tablet app, or any client, sends strokes on the work panel. They are
planned into a joint trajectory by tattotronix_control.plan - the rules the
design toolchain applies to the ROS logo - for the arm described on
/robot_description, and run on the joint trajectory controller. While it
runs, the action reports the stroke being drawn and the fraction of the time
gone, and /ink_trace shows in RViz the ink laid down so far, as the draw node
does for a stored drawing. Each drawing is its own marker, so earlier ones
stay on the panel. One drawing at a time; a cancel stops the arm where it is.
"""

import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tattotronix_interfaces.action import DrawStrokes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from tattotronix_control import arm, plan


class DrawServer(Node):

    def __init__(self):
        super().__init__("draw_server")
        self.declare_parameter("controller", "arm_controller")
        self.declare_parameter("trace_frame", "base_link")
        self.declare_parameter("settle_s", 3.0)
        group = ReentrantCallbackGroup()
        self.chain = None
        self.busy = threading.Lock()
        self.drawings = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "robot_description", self._description, latched,
                                 callback_group=group)
        self.trace = self.create_publisher(Marker, "ink_trace", 1)
        controller = self.get_parameter("controller").value
        self.client = ActionClient(self, FollowJointTrajectory,
                                   f"/{controller}/follow_joint_trajectory", callback_group=group)
        self.server = ActionServer(
            self, DrawStrokes, "draw_strokes", execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info("draw_strokes waiting for /robot_description")

    def _description(self, msg):
        try:
            self.chain = arm.Chain(msg.data)
        except Exception as e:  # a bad description must not take the node down
            self.get_logger().error(f"cannot read /robot_description: {e}")
            return
        self.get_logger().info(f"draw_strokes ready: {', '.join(self.chain.names)}")

    def _goal(self, _):
        return GoalResponse.REJECT if self.busy.locked() else GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        with self.busy:
            return self._draw(goal_handle)

    def _draw(self, gh):
        result = DrawStrokes.Result()

        def fail(why):
            self.get_logger().error(why)
            gh.abort()
            result.success = False
            result.message = why
            return result

        if self.chain is None:
            return fail("no /robot_description yet; is robot_state_publisher running?")
        strokes = []
        for i, s in enumerate(gh.request.strokes):
            if len(s.x_mm) != len(s.y_mm):
                return fail(f"stroke {i}: x_mm has {len(s.x_mm)} points, y_mm {len(s.y_mm)}")
            strokes.append(np.column_stack([s.x_mm, s.y_mm]))
        speed = gh.request.speed if gh.request.speed > 0 else 1.0

        feedback = DrawStrokes.Feedback(stage="planning", progress=0.0, stroke=0)
        gh.publish_feedback(feedback)
        try:
            p = plan.plan(self.chain, strokes, speed)
        except plan.PlanError as e:
            return fail(str(e))

        settle = float(self.get_parameter("settle_s").value)
        traj = JointTrajectory()
        traj.joint_names = self.chain.names
        # The first point gets `settle` seconds of its own, so the controller is
        # not asked to cover the way from wherever the arm is in no time at all.
        traj.points.append(JointTrajectoryPoint(positions=p.q[0].tolist(),
                                                time_from_start=_duration(settle)))
        for q, t in zip(p.q[1:], p.t[1:]):
            traj.points.append(JointTrajectoryPoint(positions=q.tolist(),
                                                    time_from_start=_duration(settle + t)))
        if not self.client.wait_for_server(timeout_sec=10.0):
            return fail("no follow_joint_trajectory server; is the controller active?")
        sent = self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))
        handle = _wait(sent, 10.0)
        if handle is None or not handle.accepted:
            return fail("the controller rejected the trajectory")

        self.drawings += 1
        ink = float(np.linalg.norm(np.diff(p.tcp, axis=0), axis=1)[p.marked[1:]].sum()) * 1000
        self.get_logger().info(
            f"drawing {len(strokes)} strokes, {ink:.0f} mm of ink, {len(p.t)} points, "
            f"{settle + p.t[-1]:.0f} s at speed {speed:g}")
        started = self.get_clock().now()
        done = handle.get_result_async()
        feedback.stage = "drawing"
        while not done.done():
            elapsed = (self.get_clock().now() - started).nanoseconds * 1e-9 - settle
            if gh.is_cancel_requested:
                handle.cancel_goal_async()
                self._trace(p, elapsed)
                gh.canceled()
                result.message = "cancelled"
                self.get_logger().info("drawing cancelled")
                return result
            feedback.progress = float(np.clip(elapsed / p.t[-1], 0.0, 1.0))
            feedback.stroke = int(max(p.stroke[min(np.searchsorted(p.t, max(elapsed, 0.0)),
                                                   len(p.t) - 1)], 0))
            gh.publish_feedback(feedback)
            self._trace(p, elapsed)
            time.sleep(0.2)
        self._trace(p, p.t[-1])
        outcome = done.result().result
        if outcome.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return fail(f"the controller stopped the drawing: "
                        f"{outcome.error_string or outcome.error_code}")
        gh.succeed()
        result.success = True
        result.seconds = (self.get_clock().now() - started).nanoseconds * 1e-9
        result.message = f"drew {len(strokes)} strokes, {ink:.0f} mm of ink"
        self.get_logger().info(f"finished: {result.message} in {result.seconds:.0f} s")
        return result

    def _trace(self, p, elapsed):
        """Ink laid down so far, as a line list in the arm frame."""
        upto = int(np.searchsorted(p.t, max(elapsed, 0.0)))
        m = Marker()
        m.header.frame_id = str(self.get_parameter("trace_frame").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "drawing"
        m.id = self.drawings
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.0008          # as the draw node draws it
        m.color.r, m.color.g, m.color.b, m.color.a = 0.13, 0.17, 0.28, 1.0
        m.pose.orientation.w = 1.0
        for i in range(1, upto):
            if p.marked[i]:
                for q in (p.tcp[i - 1], p.tcp[i]):
                    m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
        self.trace.publish(m)


def _duration(seconds):
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)) % 1_000_000_000)


def _wait(future, timeout):
    """Wait for a future that the executor's other threads complete."""
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


def main():
    rclpy.init()
    node = DrawServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
