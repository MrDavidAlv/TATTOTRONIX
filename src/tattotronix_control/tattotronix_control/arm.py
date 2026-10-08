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
The arm's kinematics at run time, read from the URDF the robot publishes.

The same chain and the same solver as docs/scripts/kinematics.py, which is the
design-time toolchain and which nothing under src/ may import: tests/
test_draw_planning.py holds the two to the same answers, so they cannot drift.
The task is the tool centre point's position plus the tool's axis direction,
five constraints against five joints.
"""

import xml.etree.ElementTree as ET

import numpy as np

CHAIN = [
    "base_mount", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
    "tool_mount_to_tool0", "tool0_to_tattoo_pen", "tattoo_pen_to_tcp",
]


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_rotation(axis, q):
    """Rodrigues, for a joint axis."""
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def homogeneous(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class Chain:
    """The serial chain from `world` to `tattoo_tcp`."""

    def __init__(self, urdf_text):
        root = ET.fromstring(urdf_text)
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.segments = []
        for name in CHAIN:
            j = joints[name]
            origin = j.find("origin")
            xyz = (np.array(origin.get("xyz", "0 0 0").split(), float)
                   if origin is not None else np.zeros(3))
            rpy = (np.array(origin.get("rpy", "0 0 0").split(), float)
                   if origin is not None else np.zeros(3))
            axis_el = j.find("axis")
            axis = np.array(axis_el.get("xyz").split(), float) if axis_el is not None else None
            limit = j.find("limit")
            lim = ((float(limit.get("lower")), float(limit.get("upper")),
                    float(limit.get("velocity", "inf")))
                   if limit is not None else None)
            self.segments.append({
                "name": name,
                "T": homogeneous(rpy_to_matrix(*rpy), xyz),
                "axis": axis,
                "type": j.get("type"),
                "limit": lim,
                "child": j.find("child").get("link"),
            })
            if axis is not None:
                # What axis_rotation and the Jacobian work out on every call,
                # worked out once: the same arithmetic, so the same answers.
                k = axis / np.linalg.norm(axis)
                K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                self.segments[-1].update(k=k, K=K, KK=K @ K)
        self.actuated = [s for s in self.segments if s["type"] == "revolute"]
        self.names = [s["name"] for s in self.actuated]
        self.lower = np.array([s["limit"][0] for s in self.actuated])
        self.upper = np.array([s["limit"][1] for s in self.actuated])
        self.velocity = np.array([s["limit"][2] for s in self.actuated])
        self.n = len(self.actuated)

    def frames(self, q):
        """Every link frame along the chain, world first."""
        q = np.asarray(q, float)
        T = np.eye(4)
        out = [("world", T)]
        i = 0
        for s in self.segments:
            T = T @ s["T"]
            if s["type"] == "revolute":
                R = np.eye(3) + np.sin(q[i]) * s["K"] + (1 - np.cos(q[i])) * s["KK"]
                T = T @ homogeneous(R, np.zeros(3))
                i += 1
            out.append((s["child"], T))
        return out

    def fk(self, q):
        return self.frames(q)[-1][1]

    def tcp(self, q):
        return self.fk(q)[:3, 3]

    def jacobian(self, q):
        """Geometric Jacobian at the TCP, 6 x n, world-aligned."""
        return self._jacobian(self.frames(q))

    def _jacobian(self, frames):
        by_link = dict(frames)
        p_e = frames[-1][1][:3, 3]
        J = np.zeros((6, self.n))
        for i, s in enumerate(self.actuated):
            T = by_link[s["child"]]
            z = T[:3, :3] @ s["k"]
            J[:3, i] = np.cross(z, p_e - T[:3, 3])
            J[3:, i] = z
        return J

    def task_jacobian(self, q):
        """
        5 x n: three position rows, two for the tool axis's direction.

        Rotation about the tool axis is not a task constraint, so that row is
        projected out.
        """
        return self._task_jacobian(self.frames(q))

    def _task_jacobian(self, frames):
        J = self._jacobian(frames)
        z = frames[-1][1][:3, 2]
        u, v = _normal_plane(z)
        Jw = J[3:, :]
        dz = np.column_stack([np.cross(Jw[:, i], z) for i in range(self.n)])
        return np.vstack([J[:3, :], u @ dz, v @ dz])

    def ik(self, p_des, z_des, q0, iters=200, tol=1e-6, damping=1e-3):
        """
        Damped least squares on the five-row task; (q, converged, residual).

        Joint limits are enforced by clamping: the panel sits well inside them,
        so a clamp that bites means the pose is out of reach.
        """
        q = np.array(q0, float)
        z_des = np.asarray(z_des, float)
        z_des = z_des / np.linalg.norm(z_des)
        for _ in range(iters):
            # One pass down the chain serves the error and the Jacobian both.
            frames = self.frames(q)
            T = frames[-1][1]
            e_p = p_des - T[:3, 3]
            z = T[:3, 2]
            u, v = _normal_plane(z)
            e_z = z_des - z
            e = np.concatenate([e_p, [u @ e_z, v @ e_z]])
            if np.linalg.norm(e) < tol:
                return q, True, float(np.linalg.norm(e))
            J = self._task_jacobian(frames)
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(5), e)
            q = np.clip(q + dq, self.lower, self.upper)
        T = self.fk(q)
        e = np.concatenate([p_des - T[:3, 3], z_des - T[:3, 2]])
        return q, False, float(np.linalg.norm(e))


def _normal_plane(z):
    """Two unit directions spanning the plane normal to z."""
    a = np.array([1.0, 0.0, 0.0])
    if abs(z @ a) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    u = np.cross(z, a)
    u /= np.linalg.norm(u)
    return u, np.cross(z, u)
