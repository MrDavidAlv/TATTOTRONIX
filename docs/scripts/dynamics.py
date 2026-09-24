"""Rigid body dynamics for the TATTOTRONIX chain.

Recursive Newton-Euler for inverse dynamics, with the mass, centre of mass and
inertia tensor of every link read from the same URDF the kinematics come from.

    tau = M(q) qdd + C(q, qd) qd + G(q)

M is recovered column by column from the unit-acceleration responses and G from
the zero-velocity, zero-acceleration response, which is exact and avoids a
second algorithm that could disagree with the first.

A caveat worth carrying into any conclusion drawn from these numbers: the link
inertias in the description are bounding-box approximations at estimated
masses, not measured mass properties. The structure of the model is right; the
magnitudes are as good as those estimates and no better.
"""

import xml.etree.ElementTree as ET

import numpy as np

from kinematics import axis_rotation, rpy_to_matrix

GRAVITY = np.array([0.0, 0.0, -9.80665])


def parse_inertials(urdf_text):
    """{link name: (mass, com in link frame, 3x3 inertia about the com)}"""
    root = ET.fromstring(urdf_text)
    out = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        m = float(inertial.find("mass").get("value"))
        o = inertial.find("origin")
        com = np.fromstring(o.get("xyz", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
        rpy = np.fromstring(o.get("rpy", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
        i = inertial.find("inertia")
        vals = {k: float(i.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
        I = np.array([[vals["ixx"], vals["ixy"], vals["ixz"]],
                      [vals["ixy"], vals["iyy"], vals["iyz"]],
                      [vals["ixz"], vals["iyz"], vals["izz"]]])
        R = rpy_to_matrix(*rpy)
        out[link.get("name")] = (m, com, R @ I @ R.T)
    return out


class Model:
    def __init__(self, chain, inertials):
        self.c = chain
        self.n = chain.n
        # One body per revolute joint, holding every link rigidly attached to
        # it. Fixed segments after the last joint are lumped onto that body, so
        # the pen's mass is felt by joint 5 rather than quietly dropped.
        self.bodies = []
        T_from_joint = None
        for s in chain.segments:
            if s["type"] == "revolute":
                self.bodies.append({"axis": s["axis"] / np.linalg.norm(s["axis"]),
                                    "pre": self._pre(s), "links": []})
                T_from_joint = np.eye(4)
                self.bodies[-1]["links"].append((s["child"], T_from_joint.copy()))
            elif self.bodies:
                T_from_joint = T_from_joint @ s["T"]
                self.bodies[-1]["links"].append((s["child"], T_from_joint.copy()))
        self.inertials = inertials
        self._lump()

    def _pre(self, seg):
        """Fixed transform from the previous joint frame to this joint's frame."""
        segs = self.c.segments
        i = segs.index(seg)
        T = seg["T"].copy()
        j = i - 1
        while j >= 0 and segs[j]["type"] != "revolute":
            T = segs[j]["T"] @ T
            j -= 1
        return T

    def _lump(self):
        """Collapse each body's links into one mass, com and inertia."""
        for b in self.bodies:
            m_tot, mc, I_tot = 0.0, np.zeros(3), np.zeros((3, 3))
            for name, T in b["links"]:
                if name not in self.inertials:
                    continue
                m, com, I = self.inertials[name]
                R, p = T[:3, :3], T[:3, 3]
                c = R @ com + p
                m_tot += m
                mc += m * c
                I_tot = I_tot + R @ I @ R.T + m * ((c @ c) * np.eye(3) - np.outer(c, c))
            com = mc / m_tot if m_tot > 0 else np.zeros(3)
            # shift the inertia back to the lumped com
            I_com = I_tot - m_tot * ((com @ com) * np.eye(3) - np.outer(com, com))
            b["m"], b["com"], b["I"] = m_tot, com, I_com

    # ---- recursive Newton-Euler --------------------------------------------

    def rnea(self, q, qd, qdd, gravity=True):
        n = self.n
        w = np.zeros(3); wd = np.zeros(3)
        vd = -GRAVITY if gravity else np.zeros(3)
        Rs, ps, ws, wds, vds, acs = [], [], [], [], [], []
        for i, b in enumerate(self.bodies):
            T = b["pre"] @ np.block([[axis_rotation(b["axis"], q[i]), np.zeros((3, 1))],
                                     [np.zeros((1, 3)), np.ones((1, 1))]])
            R, p = T[:3, :3], T[:3, 3]
            z = b["axis"]
            w_new = R.T @ w + z * qd[i]
            wd_new = R.T @ wd + np.cross(R.T @ w, z * qd[i]) + z * qdd[i]
            vd_new = R.T @ (vd + np.cross(wd, p) + np.cross(w, np.cross(w, p)))
            ac = vd_new + np.cross(wd_new, b["com"]) + np.cross(w_new, np.cross(w_new, b["com"]))
            Rs.append(R); ps.append(p)
            ws.append(w_new); wds.append(wd_new); vds.append(vd_new); acs.append(ac)
            w, wd, vd = w_new, wd_new, vd_new

        f_child = np.zeros(3)
        n_child = np.zeros(3)
        tau = np.zeros(n)
        for i in range(n - 1, -1, -1):
            b = self.bodies[i]
            F = b["m"] * acs[i]
            N = b["I"] @ wds[i] + np.cross(ws[i], b["I"] @ ws[i])
            f = F
            nn = N + np.cross(b["com"], F)
            if i + 1 < n:
                # The child's force and moment, brought into this frame, and
                # the moment that force makes about this joint.
                R_next, p_next = Rs[i + 1], ps[i + 1]
                f_next, n_next = R_next @ f_child, R_next @ n_child
                f = f + f_next
                nn = nn + n_next + np.cross(p_next, f_next)
            tau[i] = nn @ b["axis"]
            f_child, n_child = f, nn
        return tau

    def gravity(self, q):
        z = np.zeros(self.n)
        return self.rnea(q, z, z, gravity=True)

    def inertia(self, q):
        z = np.zeros(self.n)
        g0 = self.rnea(q, z, z, gravity=False)
        M = np.zeros((self.n, self.n))
        for i in range(self.n):
            e = np.zeros(self.n); e[i] = 1.0
            M[:, i] = self.rnea(q, z, e, gravity=False) - g0
        return 0.5 * (M + M.T)

    def coriolis_torque(self, q, qd):
        z = np.zeros(self.n)
        return self.rnea(q, qd, z, gravity=False)

    def forward(self, q, qd, tau):
        """qdd = M^-1 (tau - C qd - G)"""
        M = self.inertia(q)
        bias = self.rnea(q, qd, np.zeros(self.n), gravity=True)
        return np.linalg.solve(M, tau - bias)


def load(mass_model=None):
    """The chain and its dynamic model, from the live description.

    `mass_model` picks the description's inertia source, box or printed. None
    takes the description's own default, which is what every published number
    was computed with; naming it explicitly is for comparing the two.
    """
    from kinematics import Chain, URDF_XACRO
    import subprocess
    args = ["xacro", str(URDF_XACRO), "tool:=tattoo", "hardware:=none"]
    if mass_model is not None:
        args.append("mass_model:=" + mass_model)
    out = subprocess.run(args, capture_output=True, text=True, check=True)
    chain = Chain(out.stdout)
    return chain, Model(chain, parse_inertials(out.stdout))


if __name__ == "__main__":
    chain, model = load()
    print("lumped bodies:")
    for i, b in enumerate(model.bodies):
        print(f"  {chain.names[i]}  m={b['m']:.3f} kg  com={np.round(b['com'] * 1000, 1)} mm")
    q = np.zeros(model.n)
    print("\nzero pose")
    print("  gravity torque (Nm):", np.round(model.gravity(q), 4))
    M = model.inertia(q)
    print("  inertia diag (kg m^2):", np.round(np.diag(M), 5))
    print("  M condition number:", round(float(np.linalg.cond(M)), 1))
    print("\n  effort limit per joint: 20 Nm")
    print("  worst gravity torque over a random sweep:")
    worst = np.zeros(model.n)
    for _ in range(3000):
        qq = np.random.uniform(chain.lower, chain.upper)
        worst = np.maximum(worst, np.abs(model.gravity(qq)))
    print("   ", np.round(worst, 3), "Nm")
