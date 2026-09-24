# %% [markdown]
# # Dynamics of the TATTOTRONIX arm
#
# Kinematics says where the arm is. Dynamics says what it costs to get it there:
# the torque each joint must produce, given the masses it carries. This notebook
# builds the arm's equations of motion from its rigid bodies up — inertia tensors,
# the mass matrix, Coriolis and centrifugal terms, gravity — and checks each
# against the repository's recursive Newton–Euler implementation by a route that
# shares none of its code. The `assert` lines run on every build of the repository.
#
# The masses are the arm **as it was built**: printed shells, their volumes
# integrated from the meshes, with five MG996R and two SG90 servos mounted in them.
# The earlier bounding-box approximation is loaded alongside wherever the contrast
# explains something. See [mass properties](../mathematical-model/mass.md).
#
# **Contents.** 1 · The rigid bodies — 2 · The equations of motion — 3 · The mass
# matrix, and why the wrist is hard to control — 4 · Coriolis and centrifugal
# terms — 5 · Gravity — 6 · Energy, and the arm falling freely — 7 · What the
# servos have to supply

# %% [setup]

# %%
import json

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

import dynamics               # the repository's recursive Newton-Euler model
import style
from style import BLUES, C, INK2, MUTED

style.apply(**{"figure.dpi": 110})
np.set_printoptions(precision=4, suppress=True)

chain, model = nb.arm()
_, model_box = nb.arm("box")
NAMES = [s["name"] for s in chain.actuated]
G0 = 9.80665
rng = np.random.default_rng(11)

# %% [markdown]
# ## 1 · The rigid bodies
#
# Each joint moves one rigid body: the link after it, plus anything fixed to that
# link. The pen is fixed to the tool mount, so it rides on `joint_5`'s body. For
# each body the model holds a mass $m$, a centre of mass $c$ in the joint's frame,
# and an inertia tensor $I_c$ about that centre:

# %%
rows = []
for name, b in zip(NAMES, model.bodies):
    rows.append([f"`{name}`", f"{b['m'] * 1000:.1f} g",
                 ", ".join(f"{v * 1000:.1f}" for v in b["com"]),
                 ", ".join(f"{v:.2e}" for v in np.linalg.eigvalsh(b["I"]))])
nb.table(["body moved by", "mass", "centre of mass (mm)", "principal moments (kg·m²)"], rows)
print("inertia tensor of the upper arm, about its centre of mass (kg·m²):")
nb.display(nb.matrix(model.bodies[1]["I"]))

# %% [markdown]
# An inertia tensor is symmetric and positive definite, and its principal moments —
# its eigenvalues — satisfy the triangle inequality $I_1 + I_2 \ge I_3$: no rigid
# body can have one moment larger than the other two together. A tensor that broke
# it would describe nothing that exists.

# %%
for b in model.bodies:
    I1, I2, I3 = np.sort(np.linalg.eigvalsh(b["I"]))
    assert np.allclose(b["I"], b["I"].T) and I1 > 0 and I1 + I2 >= I3 * (1 - 1e-12)
print("every body's tensor is symmetric, positive definite and physically possible")

# %% [markdown]
# ### The spatial inertia matrix
#
# Rotation and translation together. For a body moving with angular velocity
# $\omega$ and with its frame origin moving at $v$, the kinetic energy is
# $\tfrac12 \mathcal V^\top \mathcal G \mathcal V$ with $\mathcal V = (\omega, v)$ and
#
# $$\mathcal G = \begin{bmatrix} I_c + m[c]^\top[c] & m[c] \\ m[c]^\top & m I \end{bmatrix}
#   \in \mathbb R^{6\times 6},$$
#
# which is the parallel-axis theorem written for velocities. It is checked against
# the energy computed the elementary way, $\tfrac12 m|v_c|^2 + \tfrac12\omega^\top I_c\omega$.

# %%
def skew(k):
    return np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])


def spatial_inertia(b):
    m, c, Ic = b["m"], b["com"], b["I"]
    G = np.zeros((6, 6))
    G[:3, :3] = Ic + m * skew(c).T @ skew(c)
    G[:3, 3:] = m * skew(c)
    G[3:, :3] = m * skew(c).T
    G[3:, 3:] = m * np.eye(3)
    return G


b = model.bodies[1]
Gs = spatial_inertia(b)
nb.display(nb.matrix(Gs))
for _ in range(100):
    w, v = rng.normal(size=3), rng.normal(size=3)
    vc = v + np.cross(w, b["com"])
    assert np.isclose(0.5 * np.r_[w, v] @ Gs @ np.r_[w, v],
                      0.5 * b["m"] * vc @ vc + 0.5 * w @ b["I"] @ w, rtol=1e-12)
print("½ Vᵀ G V equals ½ m |v_c|² + ½ ωᵀ I_c ω for 100 random twists")

# %% [markdown]
# ## 2 · The equations of motion
#
# Five joints, five equations:
#
# $$M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) = \tau .$$
#
# The repository computes them by the **recursive Newton–Euler algorithm**: a pass
# out from the base carries velocities and accelerations link by link, a pass back
# in carries forces and moments, and the joint torque is the moment about each axis.
# It gives $\tau$ for any $(q, \dot q, \ddot q)$, and every term above is read off it:
#
# - $G(q) = \mathrm{ID}(q, 0, 0)$
# - column $i$ of $M(q)$ is $\mathrm{ID}(q, 0, e_i)$ with gravity off
# - $C(q,\dot q)\,\dot q = \mathrm{ID}(q, \dot q, 0)$ with gravity off
#
# The sections below rebuild each term by a route that does not use that algorithm.

# %%
q = np.array([0.2, 0.5, 0.7, -0.3, -0.4])
qd = np.array([0.3, -0.4, 0.5, 0.8, -0.6])
qdd = np.array([-0.2, 0.3, 0.1, -0.5, 0.4])
tau = model.rnea(q, qd, qdd)
tau_split = model.inertia(q) @ qdd + model.coriolis_torque(q, qd) + model.gravity(q)
assert np.allclose(tau, tau_split, atol=1e-12)
print("τ from Newton–Euler (N·m):", np.round(tau, 5))
print("M q̈ + C q̇ + G agrees with it to", f"{np.abs(tau - tau_split).max():.1e} N·m")

# %% [markdown]
# ## 3 · The mass matrix, and why the wrist is hard to control
#
# $M(q)$ is symmetric and positive definite — kinetic energy $\tfrac12\dot q^\top M
# \dot q$ is positive for any motion — and it can be checked independently of
# Newton–Euler: it is the Hessian of kinetic energy with respect to $\dot q$, and
# kinetic energy is just the sum of each body's $\tfrac12 \mathcal V^\top \mathcal G
# \mathcal V$, with each body's velocity from the Jacobian of its own frame.

# %%
def body_jacobians(qq):
    """Twist Jacobian of each body's frame: angular rows, then linear."""
    frames = dict(chain.frames(qq))
    out = []
    for i, s in enumerate(chain.actuated):
        T = frames[s["child"]]
        J = np.zeros((6, 5))
        for j in range(i + 1):
            sj = chain.actuated[j]
            Tj = frames[sj["child"]]
            z = Tj[:3, :3] @ (sj["axis"] / np.linalg.norm(sj["axis"]))
            J[:3, j] = z
            J[3:, j] = np.cross(z, T[:3, 3] - Tj[:3, 3])
        R = T[:3, :3]
        body = np.zeros((6, 5))
        body[:3], body[3:] = R.T @ J[:3], R.T @ J[3:]    # expressed in the body's frame
        out.append(body)
    return out


def mass_matrix_from_energy(qq, mdl):
    return sum(Jb.T @ spatial_inertia(b) @ Jb for Jb, b in zip(body_jacobians(qq), mdl.bodies))


M = model.inertia(q)
M_energy = mass_matrix_from_energy(q, model)
print(f"M from kinetic energy against M from Newton–Euler: {np.abs(M - M_energy).max():.1e}")
assert np.allclose(M, M_energy, atol=1e-12)
assert np.allclose(M, M.T) and np.linalg.eigvalsh(M).min() > 0
nb.display(nb.matrix(M))

# %% [markdown]
# The diagonal says how much inertia a joint sees if every other joint is held
# still. Nothing holds them still, so the inertia a joint actually has to
# accelerate is the **effective** inertia $1/(M^{-1})_{ii}$ — what the controller's
# gains are tuned against. The off-diagonal terms are **coupling**: torque at one
# joint that accelerates another.
#
# Here is where the arm as built differs from the box approximation. Concentrating
# the mass in the servos makes every joint lighter, but not in proportion: the
# wrist's own inertia falls much further than the coupling it receives. Measured
# as the coupling a joint receives relative to its own effective inertia:

# %%
summary = json.loads((nb.REPO / "docs" / "data" / "summary.json").read_text())
q_ref = np.array(summary["q_ref_tune"])


def coupling_ratio(Mq):
    J_eff = 1 / np.diag(np.linalg.inv(Mq))
    return (np.abs(Mq).sum(axis=1) - np.abs(np.diag(Mq))) / J_eff


M_box, M_printed = model_box.inertia(q_ref), model.inertia(q_ref)
r_box, r_printed = coupling_ratio(M_box), coupling_ratio(M_printed)
nb.table(["joint", "J_eff, box (kg·m²)", "J_eff, as built", "coupling / J_eff, box", "as built",
          "change"],
         [[f"`{n}`", f"{1 / np.linalg.inv(M_box)[i, i]:.2e}",
           f"{1 / np.linalg.inv(M_printed)[i, i]:.2e}", f"{r_box[i]:.2f}", f"{r_printed[i]:.2f}",
           f"×{r_printed[i] / r_box[i]:.2f}"] for i, n in enumerate(NAMES)])
assert r_printed[3] / r_box[3] > 5          # the wrist's roll joint gains the most

fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), gridspec_kw={"width_ratios": [1, 1, 1.15]})
for ax, Mq, title in ((axes[0], M_box, "Box approximation"),
                      (axes[1], M_printed, "The arm as built")):
    D = np.sqrt(np.outer(np.diag(Mq), np.diag(Mq)))
    im = ax.imshow(np.abs(Mq) / D, cmap=style.SEQ, vmin=0, vmax=1)
    ax.set_xticks(range(5), [f"j{i + 1}" for i in range(5)])
    ax.set_yticks(range(5), [f"j{i + 1}" for i in range(5)])
    ax.set_title(title, loc="left")
    for i in range(5):
        for j in range(5):
            v = abs(Mq[i, j]) / D[i, j]
            ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=7,
                    color="white" if v > 0.55 else INK2)
fig.colorbar(im, ax=axes[:2], shrink=0.8, label="|M_ij| / √(M_ii M_jj)")
x = np.arange(5)
axes[2].bar(x - 0.2, r_box, 0.38, color=MUTED, label="box approximation")
axes[2].bar(x + 0.2, r_printed, 0.38, color=C[0], label="as built")
axes[2].set_yscale("log")
axes[2].set_xticks(x, [f"j{i + 1}" for i in range(5)])
axes[2].set_ylabel("coupling received / own effective inertia")
axes[2].set_title("How much each joint is pushed by the others", loc="left")
axes[2].legend(fontsize=8)
axes[2].grid(True, axis="y", alpha=0.9)
nb.show(fig, export="23_mass_matrix.png")

# %% [markdown]
# Two things change, and the maps show the first. The normalised coupling itself
# shifts toward the wrist: with the mass concentrated in the servos, the terms
# linking the big joints to the wrist grow. The largest changes:

# %%
def normalised(Mq):
    return np.abs(Mq) / np.sqrt(np.outer(np.diag(Mq), np.diag(Mq)))


Nb, Np = normalised(M_box), normalised(M_printed)
pairs = sorted(((Np[i, j] - Nb[i, j], i, j) for i in range(5) for j in range(i + 1, 5)),
               reverse=True)[:3]
for d, i, j in pairs:
    print(f"  j{i + 1}–j{j + 1}: {Nb[i, j]:.2f} → {Np[i, j]:.2f}")
assert Np[0, 3] > 3 * Nb[0, 3]           # joint_1 to joint_4, the largest shift

# %% [markdown]
# The second is the bar chart: relative to what each joint carries itself, the
# coupling it receives grows most at the wrist. A controller that treats each
# joint on its own — as a PID per axis does — sees that coupling as a
# disturbance, and a light joint is pushed further by the same disturbance. That
# is consistent with the wrist being where tracking degraded when the masses
# changed ([control](../mathematical-model/control.md#5-what-actually-helps)); it
# is not a proof of it.

# %% [markdown]
# ## 4 · Coriolis and centrifugal terms
#
# Velocity-dependent torques come from how $M$ changes along the motion. With the
# **Christoffel symbols** of the first kind,
#
# $$c_{ijk} = \tfrac12\left(\frac{\partial M_{ij}}{\partial q_k} +
#   \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right),
#   \qquad C_{ij}(q,\dot q) = \sum_k c_{ijk}\,\dot q_k .$$
#
# Built this way — from derivatives of $M$ alone — $C\dot q$ must equal the
# velocity terms Newton–Euler produces directly. And $\dot M - 2C$ must be
# skew-symmetric: the property that says these forces do no work, which is what
# makes the arm's energy balance close.

# %%
def dM_dq(qq, h=1e-6):
    return np.array([(model.inertia(qq + h * e) - model.inertia(qq - h * e)) / (2 * h)
                     for e in np.eye(5)])


def coriolis_matrix(qq, qqd):
    dM = dM_dq(qq)                       # dM[k] = dM / dq_k
    Cm = np.zeros((5, 5))
    for i in range(5):
        for j in range(5):
            Cm[i, j] = 0.5 * sum((dM[k][i, j] + dM[j][i, k] - dM[i][j, k]) * qqd[k]
                                 for k in range(5))
    return Cm, dM


Cm, dM = coriolis_matrix(q, qd)
c_rnea = model.coriolis_torque(q, qd)
print("C q̇ from Christoffel symbols against Newton–Euler: "
      f"{np.abs(Cm @ qd - c_rnea).max():.1e} "
      f"N·m, on torques of {np.abs(c_rnea).max():.1e}")
assert np.abs(Cm @ qd - c_rnea).max() < 1e-6 * np.abs(c_rnea).max() + 1e-12
N = sum(dM[k] * qd[k] for k in range(5)) - 2 * Cm
print(f"Ṁ − 2C is skew-symmetric: |N + Nᵀ| = {np.abs(N + N.T).max():.1e}")
assert np.abs(N + N.T).max() < 1e-12
nb.display(nb.matrix(Cm))

# %% [markdown]
# ## 5 · Gravity
#
# Gravity is the gradient of potential energy, $G(q) = \partial U/\partial q$ with
# $U = \sum m_i\, g\, z_{c,i}(q)$. Newton–Euler gets it from forces and moments; the
# gradient gets it from a scalar. Two unrelated routes, compared at the poses the
# documentation quotes and at random ones:

# %%
worst = max(dynamics.gravity_check(model, qq)
            for qq in [*dynamics.CHECK_POSES, *rng.uniform(-1.5, 1.5, (20, 5))])
print(f"Newton–Euler gravity against dU/dq, 23 poses: worst {worst:.1e} N·m")
assert worst < 1e-8
g0 = model.gravity(np.zeros(5))
print("gravity torque at the zero pose (N·m):", np.round(g0, 5))
upper = model.bodies[1]
assert np.isclose(g0[1] - g0[2], -G0 * upper["m"] * upper["com"][0], rtol=1e-9, atol=1e-15)
print("joint_2 and joint_3 differ by exactly the upper arm's own moment, -g·m·x")

# %% [markdown]
# ## 6 · Energy, and the arm falling freely
#
# With no torque applied, $M\ddot q = -C\dot q - G$: the arm falls under its own
# weight. Integrating that with fourth-order Runge–Kutta from a raised pose, total
# energy $\tfrac12\dot q^\top M\dot q + U$ must stay constant, because nothing in
# the model dissipates it. How well it does is a test of everything above at once.
#
# The model knows only rigid-body dynamics: no joint limits, no collisions, no
# friction. So the arm swings through poses the real one could not reach — and
# the wrist, the lightest part, spins fastest, pushed by the same coupling that
# makes it hard to control.

# %%
def step_rk4(x, dt):
    def f(s):
        return np.r_[s[5:], model.forward(s[:5], s[5:], np.zeros(5))]
    k1 = f(x)
    k2 = f(x + dt / 2 * k1)
    k3 = f(x + dt / 2 * k2)
    k4 = f(x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def energy(x):
    kinetic = 0.5 * x[5:] @ model.inertia(x[:5]) @ x[5:]
    return kinetic, dynamics.potential(model, x[:5])


dt, T = 2e-3, 0.8
x = np.r_[np.array([0.0, 0.3, 0.4, 0.2, 0.5]), np.zeros(5)]
states, E = [x], [energy(x)]
for _ in range(int(T / dt)):
    x = step_rk4(x, dt)
    states.append(x)
    E.append(energy(x))
states, E = np.array(states), np.array(E)
tt = np.arange(len(states)) * dt
total = E.sum(axis=1)
drift = np.abs(total - total[0]).max()
print(f"energy drift over {T} s: {drift:.1e} J, on {np.ptp(E[:, 1]):.3f} J exchanged "
      f"between kinetic and potential")
assert drift < 1e-4 * np.ptp(E[:, 1])
peak_speed = np.abs(states[:, 5:]).max(axis=0)
print("peak joint speed during the fall (rad/s):",
      ", ".join(f"{n} {v:.1f}" for n, v in zip(NAMES, peak_speed)))
fastest = NAMES[int(np.argmax(peak_speed))]
assert set(np.argsort(peak_speed)[-2:]) == {3, 4}, "the two wrist joints should be fastest"

fig, (a1, a2) = plt.subplots(1, 2, figsize=(11.5, 3.9))
a1.plot(tt, E[:, 0] - E[0, 0], color=C[1], label="kinetic")
a1.plot(tt, E[:, 1] - E[0, 1], color=C[0], label="potential")
a1.plot(tt, total - total[0], color=INK2, lw=2.0, label="total")
a1.set_xlabel("time (s)")
a1.set_ylabel("change from the start (J)")
a1.set_title("Energy while the arm falls with no torque applied", loc="left")
a1.legend(fontsize=8)
a1.grid(True, alpha=0.9)
for i in range(5):
    a2.plot(tt, states[:, i], color=C[i], label=NAMES[i])
a2.set_xlabel("time (s)")
a2.set_ylabel("joint angle (rad)")
a2.set_title(f"Joint angles: {fastest} reaches the highest speed", loc="left")
a2.legend(fontsize=8, ncol=2)
a2.grid(True, alpha=0.9)
nb.show(fig, export="24_free_fall.png")

# %% [markdown]
# The fall, side view:

# %%
links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_link",
         "tool_mount_link", "tool0", "tattoo_tcp"]
fig, ax = plt.subplots(figsize=(6, 5))
arm_line, = ax.plot([], [], "-o", color=INK2, lw=2.2, ms=4)
clock = ax.text(-0.33, 0.33, "", fontsize=9)
ax.set_xlim(-0.35, 0.35)
ax.set_ylim(-0.3, 0.36)
ax.set_aspect("equal")
ax.axhline(0, color=MUTED, lw=0.8)
ax.grid(True, alpha=0.9)
ax.set_xlabel("x (m)")
ax.set_ylabel("z (m)")
frames_i = np.arange(0, len(states), 4)


def frame_fall(k):
    f = dict(chain.frames(states[frames_i[k], :5]))
    pts = np.array([f[n][:3, 3] for n in links])
    arm_line.set_data(pts[:, 0], pts[:, 2])
    clock.set_text(f"t = {tt[frames_i[k]]:.2f} s")
    return arm_line, clock


nb.animate(animation.FuncAnimation(fig, frame_fall, frames=len(frames_i), interval=40), fps=25)

# %% [markdown]
# ## 7 · What the servos have to supply
#
# While the arm draws, the torque that holds it against gravity and carries it
# along the path is $G(q) + C(q,\dot q)\dot q$. That is only part of what the
# servos give — the feedback that corrects errors adds to it, and the reference
# has instantaneous changes of velocity at every corner that a real controller
# spreads out — but it is the floor, and it is set by the masses alone.
#
# Against it, what each joint's servos can give at most: their catalogue stall
# torque, two MG996R on the shoulder. A servo sustains only a fraction of its stall
# torque, which hobby catalogues do not state, so the margin that matters is
# smaller than the one drawn.

# %%
traj = nb.trajectory("ros_logo")
Q, t = traj["q"][::5], traj["t"][::5]
Qd = np.gradient(Q, t, axis=0)
tau_path = np.array([model.gravity(a) + model.coriolis_torque(a, b) for a, b in zip(Q, Qd)])

mass = json.loads((nb.REPO / "docs" / "data" / "mass_properties.json").read_text())
servos = mass["declared"]["servos"]
layout = {"joint_1": ["large"], "joint_2": ["large", "large"], "joint_3": ["large"],
          "joint_4": ["large"], "joint_5": ["small"]}
stall = np.array([sum(servos[k]["stall_Nm"] for k in layout[n]) for n in NAMES])
peak = np.abs(tau_path).max(axis=0)
nb.table(["joint", "servos", "peak |G + C q̇| on the drawing", "stall torque available",
          "share of stall"],
         [[f"`{n}`", " + ".join(servos[k]["part"] for k in layout[n]), f"{peak[i]:.4f} N·m",
           f"{stall[i]:.2f} N·m", f"{100 * peak[i] / stall[i]:.1f}%"]
          for i, n in enumerate(NAMES)])

fig, ax = plt.subplots(figsize=(8, 3.8))
x = np.arange(5)
ax.bar(x, stall, 0.6, color=BLUES[1], label="stall torque of the servos (catalogue)")
ax.bar(x, peak, 0.36, color=C[0], label="peak G + C q̇ while drawing")
for i in range(5):
    share = 100 * peak[i] / stall[i]
    # Just above each bar, multiplicatively: on a log axis a fixed offset leaves
    # the small bars' labels floating decades away from them.
    ax.text(x[i], peak[i] * 1.4, "<0.1%" if share < 0.1 else f"{share:.1f}%", ha="center",
            fontsize=8, color=INK2)
ax.set_yscale("log")
ax.set_xticks(x, [f"{n}\n{' + '.join(servos[k]['part'] for k in layout[n])}" for n in NAMES],
              fontsize=8)
ax.set_ylabel("torque (N·m)")
ax.set_title("Gravity and velocity torque while drawing, against what the servos can give",
             loc="left")
ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
ax.grid(True, axis="y", alpha=0.9)
nb.show(fig, export="25_servo_torque.png")
