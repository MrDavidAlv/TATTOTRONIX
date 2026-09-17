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
Drive the arm through a solved trajectory, and show the ink.

The whole chain - artwork, mask, contour, fill, inverse kinematics - is solved
offline by docs/scripts and exported into config/trajectories/. This node sends
one of those to `joint_trajectory_controller` and, while it runs, publishes the
part of the drawing the needle has already laid down as an RViz marker.

Which drawing is the `art` parameter, a file name in that directory without the
extension. The node lists what is installed when given a name it does not have.

The marker is why the drawing is visible at all: neither Gazebo nor RViz leaves
a mark when a tool passes over a surface, so without it the arm moves and
nothing appears. The trace is drawn from the *planned* tip positions, gated on
elapsed time, which makes it a picture of the commanded path rather than the
achieved one. That distinction matters and is why the marker is not evidence of
accuracy - the tracking error is measured in docs/mathematical-model/control.md,
not here.

    ros2 run tattotronix_control draw
    ros2 run tattotronix_control draw --ros-args -p art:=semillero
"""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

TRAJECTORIES = "trajectories"
DEFAULT_ART = "ros_logo"


def _duration(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class Draw(Node):

    def __init__(self):
        super().__init__("draw")
        self.declare_parameter("art", DEFAULT_ART)
        self.declare_parameter("controller", "arm_controller")
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("trace_frame", "base_link")
        self.declare_parameter("settle_s", 3.0)

        speed = float(self.get_parameter("speed").value)
        if speed <= 0.0:
            raise ValueError("speed must be positive")

        art = str(self.get_parameter("art").value)
        root = Path(get_package_share_directory("tattotronix_control")) / "config" / TRAJECTORIES
        path = root / (art if art.endswith(".npz") else art + ".npz")
        if not path.exists():
            have = sorted(p.stem for p in root.glob("*.npz"))
            raise FileNotFoundError(
                f"no trajectory named {art!r} in {root}. "
                f"Available: {', '.join(have) if have else 'none'}. "
                "Export one with docs/scripts/export_trajectory.py, then rebuild "
                "tattotronix_control.")
        d = np.load(path, allow_pickle=False)
        self.joints = [str(j) for j in d["joints"]]
        self.q = d["q"].astype(float)
        self.t = d["t"].astype(float) / speed
        self.marked = d["marked"]
        self.tcp = d["tcp"].astype(float)

        src = str(d["source"]) if "source" in d else art
        self.get_logger().info(
            f"drawing {art}: {src}")
        self.get_logger().info(
            f"{len(self.q)} points, {self.t[-1]:.0f} s at speed {speed:g}, "
            f"{int(self.marked.sum())} of them marking")

        self.trace = self.create_publisher(Marker, "ink_trace", 1)
        self.client = ActionClient(
            self, FollowJointTrajectory,
            f"/{self.get_parameter('controller').value}/follow_joint_trajectory")
        self.start_time = None
        self.create_timer(0.1, self._publish_trace)
        self._send()

    def _send(self):
        settle = float(self.get_parameter("settle_s").value)
        self.get_logger().info("waiting for the controller action server...")
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("no action server; is the controller spawned?")

        traj = JointTrajectory()
        traj.joint_names = self.joints
        # A first point at the start pose, given `settle` seconds of its own.
        # Without it the controller has to cover the gap between wherever the
        # arm is parked and the first path point inside that point's own time
        # slice, which for the first point is zero.
        traj.points.append(JointTrajectoryPoint(
            positions=self.q[0].tolist(), time_from_start=_duration(settle)))
        for qi, ti in zip(self.q[1:], self.t[1:]):
            traj.points.append(JointTrajectoryPoint(
                positions=qi.tolist(), time_from_start=_duration(settle + ti)))

        goal = FollowJointTrajectory.Goal(trajectory=traj)
        self.get_logger().info(
            f"sending {len(traj.points)} points, {settle + self.t[-1]:.0f} s")
        self.client.send_goal_async(goal).add_done_callback(self._accepted)

    def _accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("the controller rejected the trajectory")
            rclpy.shutdown()
            return
        self.start_time = self.get_clock().now()
        self.get_logger().info("drawing")
        handle.get_result_async().add_done_callback(self._done)

    def _done(self, future):
        self.get_logger().info(f"finished: {future.result().result.error_string or 'ok'}")
        self._publish_trace()

    def _publish_trace(self):
        """Ink laid down so far, as a line list in the arm frame."""
        if self.start_time is None:
            return
        settle = float(self.get_parameter("settle_s").value)
        now = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 - settle
        upto = int(np.searchsorted(self.t, max(now, 0.0)))

        m = Marker()
        m.header.frame_id = str(self.get_parameter("trace_frame").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "logo"
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.0008          # the 0.3 mm line, rendered a little wider to read
        m.color.r, m.color.g, m.color.b, m.color.a = 0.13, 0.17, 0.28, 1.0
        m.pose.orientation.w = 1.0
        # One segment per marking step. A line list rather than a strip, because
        # a strip would join the end of one stroke to the start of the next and
        # draw the travel moves as ink.
        for i in range(1, upto):
            if not self.marked[i]:
                continue
            for p in (self.tcp[i - 1], self.tcp[i]):
                m.points.append(_point(p))
        self.trace.publish(m)


def _point(p):
    from geometry_msgs.msg import Point
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def main():
    rclpy.init()
    node = Draw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
