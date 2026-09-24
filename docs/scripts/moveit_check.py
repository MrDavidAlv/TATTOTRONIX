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
Ask a running move_group what docs/moveit.md claims it answers.

A joint-space plan, position-only IK onto the middle of the panel, and state
validity at the poses the document lists, written to docs/data/moveit_check.json
so the document quotes a run rather than a memory of one. Start it with
docs/scripts/moveit_check.sh, which brings move_group up on mock hardware.
"""

import json
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PositionIKRequest, RobotState)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

OUT = Path(__file__).resolve().parents[1] / "data" / "moveit_check.json"
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GOAL = [0.4, -0.3, 0.5, 0.1, -0.2]
PANEL_X, PANEL_Z = 0.21, 0.005          # analysis.py's panel centre
POSES = {"home": [0, 0, 0, 0, 0], "joint_2 -0.5": [0, -0.5, 0, 0, 0],
         "joint_2 +0.5": [0, 0.5, 0, 0, 0], "j2 0.5, j3 -0.8": [0, 0.5, -0.8, 0, 0]}


def call(node, client, request, timeout=30.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def main():
    rclpy.init()
    node = Node("moveit_check")
    plan = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    valid = node.create_client(GetStateValidity, "/check_state_validity")
    for c in (plan, ik, valid):
        if not c.wait_for_service(timeout_sec=30.0):
            sys.exit(f"{c.srv_name} never appeared")
    end = node.get_clock().now().nanoseconds + 5e9           # let state arrive
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {}

    req = MotionPlanRequest(group_name="arm", allowed_planning_time=10.0,
                            num_planning_attempts=10)
    goal = Constraints()
    for name, v in zip(JOINTS, GOAL):
        goal.joint_constraints.append(JointConstraint(
            joint_name=name, position=float(v), tolerance_above=0.01,
            tolerance_below=0.01, weight=1.0))
    req.goal_constraints.append(goal)
    r = call(node, plan, GetMotionPlan.Request(motion_plan_request=req)).motion_plan_response
    pts = r.trajectory.joint_trajectory.points
    out["plan"] = {"success": r.error_code.val == 1, "waypoints": len(pts),
                   "duration_s": (pts[-1].time_from_start.sec
                                  + pts[-1].time_from_start.nanosec * 1e-9) if pts else None,
                   "planning_time_s": r.planning_time}

    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x, ps.pose.position.z = PANEL_X, PANEL_Z
    ps.pose.orientation.w = 1.0
    ikr = PositionIKRequest(group_name="arm", pose_stamped=ps, ik_link_name="tattoo_tcp")
    ikr.timeout.sec = 2
    r = call(node, ik, GetPositionIK.Request(ik_request=ikr))
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    out["panel_ik"] = {"success": r.error_code.val == 1,
                       "q": [sol.get(j) for j in JOINTS]}

    out["validity"] = {}
    for label, q in POSES.items():
        rs = RobotState(joint_state=JointState(name=JOINTS, position=[float(v) for v in q]))
        r = call(node, valid, GetStateValidity.Request(robot_state=rs, group_name="arm"))
        out["validity"][label] = {"valid": r.valid, "contacts": sorted(
            {" / ".join(sorted((c.contact_body_1, c.contact_body_2))) for c in r.contacts})}

    OUT.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if out["plan"]["success"] and out["panel_ik"]["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
