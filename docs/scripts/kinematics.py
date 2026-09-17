"""TATTOTRONIX kinematics, read straight off the description.

Nothing here hardcodes a joint origin. The chain is parsed from the URDF that
xacro emits, so this module and the robot cannot drift apart: if a joint moves
in the xacro, every number in DEVELOPMENT.md moves with it on the next run.

Frames follow the URDF: +z of `tool0` is the tool axis, `tattoo_tcp` sits at the
pen tip. The task the arm has to serve is a pen tip position plus a pen axis
direction, which is 3 + 2 = 5 constraints against 5 joints.
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
URDF_XACRO = REPO / "src/tattotronix_description/urdf/tattotronix.urdf.xacro"

# The chain the tool rides on, root first.
CHAIN = [
    "base_mount", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
    "tool_mount_to_tool0", "tool0_to_tattoo_pen", "tattoo_pen_to_tcp",
]


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_rotation(axis, q):
    """Rodrigues, for a unit joint axis."""
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
            xyz = (np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                   if origin is not None else np.zeros(3))
            rpy = (np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                   if origin is not None else np.zeros(3))
            axis_el = j.find("axis")
            axis = np.fromstring(axis_el.get("xyz"), sep=" ") if axis_el is not None else None
            limit = j.find("limit")
            lim = ((float(limit.get("lower")), float(limit.get("upper")))
                   if limit is not None else None)
            self.segments.append({
                "name": name,
                "T": homogeneous(rpy_to_matrix(*rpy), xyz),
                "axis": axis,
                "type": j.get("type"),
                "limit": lim,
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
            })
        self.actuated = [s for s in self.segments if s["type"] == "revolute"]
        self.names = [s["name"] for s in self.actuated]
        self.lower = np.array([s["limit"][0] for s in self.actuated])
        self.upper = np.array([s["limit"][1] for s in self.actuated])
        self.n = len(self.actuated)

    @classmethod
    def from_description(cls, tool="tattoo"):
        out = subprocess.run(
            ["xacro", str(URDF_XACRO), f"tool:={tool}", "hardware:=none"],
            capture_output=True, text=True, check=True,
        )
        return cls(out.stdout)

    # ---- forward kinematics -------------------------------------------------

    def frames(self, q):
        """Every link frame along the chain, world first."""
        q = np.asarray(q, float)
        T = np.eye(4)
        out = [("world", T.copy())]
        i = 0
        for s in self.segments:
            T = T @ s["T"]
            if s["type"] == "revolute":
                T = T @ homogeneous(axis_rotation(s["axis"], q[i]), np.zeros(3))
                i += 1
            out.append((s["child"], T.copy()))
        return out

    def fk(self, q):
        return self.frames(q)[-1][1]

    def tcp(self, q):
        return self.fk(q)[:3, 3]

    def tool_axis(self, q):
        """Unit vector the pen points along, in world."""
        return self.fk(q)[:3, 2]

    # ---- differential kinematics -------------------------------------------

    def jacobian(self, q):
        """Geometric Jacobian at the TCP, 6 x n, world-aligned."""
        frames = self.frames(q)
        by_link = dict(frames)
        p_e = frames[-1][1][:3, 3]
        J = np.zeros((6, self.n))
        for i, s in enumerate(self.actuated):
            T = by_link[s["child"]]
            z = T[:3, :3] @ (s["axis"] / np.linalg.norm(s["axis"]))
            p = T[:3, 3]
            J[:3, i] = np.cross(z, p_e - p)
            J[3:, i] = z
        return J

    def manipulability(self, q):
        """Yoshikawa measure on the 5-row task Jacobian (position + axis tilt)."""
        J = self.task_jacobian(q)
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def task_jacobian(self, q):
        """5 x n: three position rows, two rows for the pen-axis direction.

        Rotation about the pen axis is not a task constraint - the needle is
        round - so that row is projected out rather than left to fight the
        solver for the one degree of freedom the arm does not have.
        """
        J = self.jacobian(q)
        z = self.tool_axis(q)
        # two directions spanning the plane normal to the pen axis
        a = np.array([1.0, 0.0, 0.0])
        if abs(z @ a) > 0.9:
            a = np.array([0.0, 1.0, 0.0])
        u = np.cross(z, a)
        u /= np.linalg.norm(u)
        v = np.cross(z, u)
        # d(z)/dq = omega x z  ->  rows u.(omega x z), v.(omega x z)
        Jw = J[3:, :]
        dz = np.column_stack([np.cross(Jw[:, i], z) for i in range(self.n)])
        return np.vstack([J[:3, :], u @ dz, v @ dz])

    # ---- inverse kinematics -------------------------------------------------

    def ik(self, p_des, z_des, q0, iters=200, tol=1e-6, damping=1e-3):
        """Damped least squares on the 5-row task.

        Returns (q, converged, residual). Joint limits are enforced by
        clamping, which is honest for this arm: every axis is +-1.57 and the
        panel sits well inside, so a clamp that bites means the pose is
        genuinely unreachable rather than that the solver needs coaxing.
        """
        q = np.array(q0, float)
        z_des = np.asarray(z_des, float)
        z_des = z_des / np.linalg.norm(z_des)
        for _ in range(iters):
            T = self.fk(q)
            e_p = p_des - T[:3, 3]
            z = T[:3, 2]
            # orientation error as the rotation that takes z onto z_des,
            # expressed in the two task directions
            a = np.array([1.0, 0.0, 0.0])
            if abs(z @ a) > 0.9:
                a = np.array([0.0, 1.0, 0.0])
            u = np.cross(z, a)
            u /= np.linalg.norm(u)
            v = np.cross(z, u)
            e_z = z_des - z
            e = np.concatenate([e_p, [u @ e_z, v @ e_z]])
            if np.linalg.norm(e) < tol:
                return q, True, float(np.linalg.norm(e))
            J = self.task_jacobian(q)
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(5), e)
            q = np.clip(q + dq, self.lower, self.upper)
        T = self.fk(q)
        e_p = p_des - T[:3, 3]
        e_z = z_des - T[:3, 2]
        return q, False, float(np.linalg.norm(np.concatenate([e_p, e_z])))


def load():
    return Chain.from_description()


if __name__ == "__main__":
    c = load()
    print("actuated joints:", c.names)
    print("limits (rad):", list(zip(np.round(c.lower, 3), np.round(c.upper, 3))))
    q0 = np.zeros(c.n)
    print("\nzero pose")
    for name, T in c.frames(q0):
        print(f"  {name:<17} {np.round(T[:3, 3] * 1000, 2)} mm")
    print("\n  TCP      ", np.round(c.tcp(q0) * 1000, 2), "mm")
    print("  tool axis", np.round(c.tool_axis(q0), 4))
    print("  manipulability", round(c.manipulability(q0), 6))
