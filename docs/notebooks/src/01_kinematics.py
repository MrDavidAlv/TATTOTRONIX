# %% [markdown]
# # Kinematics of the TATTOTRONIX arm
#
# TATTOTRONIX is a five-axis desktop manipulator that draws the official ROS logo
# with a tattoo needle. This notebook builds its kinematics from first
# principles — rotation matrices, homogeneous transforms, the product of
# exponentials, Jacobians, singular values, inverse kinematics — and checks every
# step against the model the repository actually runs. Each formula is shown to
# be right, not merely stated: the `assert` lines are executed on every build of
# the repository, so if one of them failed, this notebook would not have been
# published.
#
# The robot is read from its URDF, the same description `robot_state_publisher`
# loads. No joint origin is typed in anywhere below.
#
# **Contents.** 1 · The chain — 2 · Rotations — 3 · Homogeneous transforms and
# forward kinematics — 4 · The product of exponentials — 5 · Velocity kinematics
# and Jacobians — 6 · Singularities and manipulability — 7 · Workspace —
# 8 · Inverse kinematics — 9 · The arm drawing, in 2D and 3D
#
# Companion documents: [kinematics](../mathematical-model/kinematics.md) and the
# [parameter reference](../mathematical-model/parameters.md).

# %% [setup]

# %%
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.linalg import expm

import kinematics             # the repository's forward and inverse kinematics
import style
from style import C, INK2, MUTED

style.apply(**{"figure.dpi": 110})
np.set_printoptions(precision=4, suppress=True)
sp.init_printing()

chain, model = nb.arm()
rng = np.random.default_rng(7)
LOWER = np.array([s["limit"][0] for s in chain.actuated])
UPPER = np.array([s["limit"][1] for s in chain.actuated])

# %% [markdown]
# ## 1 · The chain
#
# Five revolute joints, then three fixed frames: the tool flange `tool0`, the pen,
# and the needle tip `tattoo_tcp`, where every Cartesian number in this project is
# measured. Each row is read from the URDF.

# %%
rows = []
for s in chain.segments:
    axis = "—" if s["axis"] is None else "(" + ", ".join(f"{a:g}" for a in s["axis"]) + ")"
    lim = "—" if s["limit"] is None else f"±{s['limit'][1]:.2f} rad"
    xyz = ", ".join(f"{v * 1000:.2f}" for v in s["T"][:3, 3])
    rows.append([f"`{s['name']}`", f"{s['parent']} → {s['child']}", s["type"], axis, xyz, lim])
nb.table(["joint", "parent → child", "type", "axis", "origin from parent (mm)", "limit"], rows)

# %% [markdown]
# ## 2 · Rotations
#
# A rotation matrix $R \in SO(3)$ holds, in its columns, the axes of a rotated frame
# expressed in the fixed one. Two properties define the group: $R^\top R = I$ and
# $\det R = +1$. The three elementary rotations:

# %%
th = sp.symbols("theta", real=True)


def Rx(a):
    return sp.Matrix([[1, 0, 0], [0, sp.cos(a), -sp.sin(a)], [0, sp.sin(a), sp.cos(a)]])


def Ry(a):
    return sp.Matrix([[sp.cos(a), 0, sp.sin(a)], [0, 1, 0], [-sp.sin(a), 0, sp.cos(a)]])


def Rz(a):
    return sp.Matrix([[sp.cos(a), -sp.sin(a), 0], [sp.sin(a), sp.cos(a), 0], [0, 0, 1]])


nb.display(Rx(th), Ry(th), Rz(th))
for R in (Rx(th), Ry(th), Rz(th)):
    assert sp.simplify(R.T * R) == sp.eye(3) and sp.simplify(R.det()) == 1

# %% [markdown]
# ### Rotation about any axis: Rodrigues' formula
#
# A joint turns about its axis $\hat k$. With $K = [\hat k]_\times$, the
# skew-symmetric matrix that computes $\hat k \times x$,
#
# $$R(\hat k, \theta) = I + \sin\theta\,K + (1 - \cos\theta)\,K^2 = e^{K\theta}.$$
#
# It is the matrix exponential of an element of $\mathfrak{so}(3)$, and on the three
# coordinate axes it reduces to the elementary rotations above — checked
# symbolically. `kinematics.axis_rotation` is the repository's implementation;
# it is checked against `scipy.linalg.expm` on random axes.

# %%
def skew(k):
    return sp.Matrix([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])


def rodrigues(k, a):
    K = skew(k)
    return sp.eye(3) + sp.sin(a) * K + (1 - sp.cos(a)) * K ** 2


nb.display(rodrigues(sp.Matrix(sp.symbols("k_x k_y k_z")), th))
for axis, R in ((sp.Matrix([1, 0, 0]), Rx), (sp.Matrix([0, 1, 0]), Ry),
                (sp.Matrix([0, 0, 1]), Rz)):
    assert sp.simplify(rodrigues(axis, th) - R(th)) == sp.zeros(3, 3)


def skew_np(k):
    return np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])


worst = 0.0
for _ in range(200):
    k = rng.normal(size=3)
    k /= np.linalg.norm(k)
    a = rng.uniform(-np.pi, np.pi)
    R = kinematics.axis_rotation(k, a)
    worst = max(worst, np.abs(R - expm(skew_np(k) * a)).max())
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12) and np.isclose(np.linalg.det(R), 1)
print(f"Rodrigues against the matrix exponential, 200 random axes: worst {worst:.1e}")
assert worst < 1e-12

# %% [markdown]
# ### The URDF's roll–pitch–yaw
#
# A URDF gives each fixed rotation as roll, pitch, yaw about the *fixed* axes, in
# that order, which composes as $R = R_z(\psi)\,R_y(\theta)\,R_x(\phi)$. The tool
# flange uses it: `rpy = (0, π/2, 0)` turns the bracket's $+x$ into the tool
# axis.

# %%
phi, theta_, psi = sp.symbols("phi theta psi", real=True)
R_rpy = Rz(psi) * Ry(theta_) * Rx(phi)
nb.display(R_rpy)
rpy_num = sp.lambdify((phi, theta_, psi), R_rpy, "numpy")
for r, p, y in rng.uniform(-np.pi, np.pi, (100, 3)):
    assert np.allclose(np.array(rpy_num(r, p, y), float), kinematics.rpy_to_matrix(r, p, y),
                       atol=1e-12)
print("kinematics.rpy_to_matrix is Rz·Ry·Rx: 100 random angles agree")

# %% [markdown]
# ## 3 · Homogeneous transforms and forward kinematics
#
# Position and orientation together:
#
# $$T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3), \qquad
#   T^{-1} = \begin{bmatrix} R^\top & -R^\top p \\ 0 & 1 \end{bmatrix}.$$
#
# Each segment of the chain is a fixed transform — the joint's origin in its
# parent — followed, for a revolute joint, by the rotation about its axis:
#
# $$T_i(q_i) = T_i^{\text{fixed}}\; \text{Rot}(\hat k_i, q_i).$$
#
# That is exactly what the URDF means and what `robot_state_publisher` computes,
# which is why this project uses it rather than Denavit–Hartenberg: there is no
# convention to translate into, so nowhere for the model and the robot to
# disagree. The offsets below are exact rationals taken from the URDF.

# %%
def H(R, p=(0, 0, 0)):
    T = sp.eye(4)
    T[:3, :3] = R
    T[:3, 3] = sp.Matrix(p)
    return T


def exact(M):
    return sp.Matrix(np.round(M, 9).tolist()).applyfunc(sp.nsimplify)


q = sp.symbols("q1:6", real=True)
segments, i = [], 0
for s in chain.segments:
    Ti = exact(s["T"])
    if s["type"] == "revolute":
        Ti = Ti * H(rodrigues(sp.Matrix(np.round(s["axis"]).astype(int).tolist()), q[i]))
        i += 1
    segments.append((s["name"], Ti))

for name, Ti in segments[1:6]:
    print(name)
    nb.display(Ti.evalf(4))

# %% [markdown]
# Multiplying them all gives the needle tip as a function of the five joint
# angles. The position is short enough to read; the third column of the rotation
# is the direction the needle points.

# %%
T_total = sp.eye(4)
for _, Ti in segments:
    T_total = T_total * Ti
p_tip = T_total[:3, 3].applyfunc(lambda e: sp.trigsimp(sp.expand(e)))
z_tip = T_total[:3, 2].applyfunc(lambda e: sp.trigsimp(sp.expand(e)))
nb.display(p_tip)
nb.display(z_tip)

fk_p = sp.lambdify(q, p_tip, "numpy")
fk_z = sp.lambdify(q, z_tip, "numpy")
worst = 0.0
for qq in rng.uniform(LOWER, UPPER, (200, 5)):
    T = chain.fk(qq)
    worst = max(worst, np.abs(np.array(fk_p(*qq), float).ravel() - T[:3, 3]).max(),
                np.abs(np.array(fk_z(*qq), float).ravel() - T[:3, 2]).max())
print(f"symbolic forward kinematics against the repository's chain, 200 poses: "
      f"worst {worst:.1e}")
assert worst < 1e-12

print("tip at the zero pose, mm:", np.round(chain.tcp(np.zeros(5)) * 1000, 2))

# %% [markdown]
# ### Every frame of the chain
#
# The same transforms, drawn: the arm at a pose from the drawing, with the frame of
# every joint and of the needle tip. $x$, $y$, $z$ are red, green and blue, the
# usual convention.

# %%
traj = nb.trajectory("ros_logo")
q_draw = traj["q"][len(traj["q"]) // 2]
parts = nb.visual_parts()
frames = dict(chain.frames(q_draw))

fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(projection="3d")
ax.computed_zorder = False           # draw the frames over the mesh, not behind it
nb.draw_arm(ax, nb.posed(parts, chain, q_draw), alpha=0.22)
AXC = [C[7], C[2], C[0]]
# Label offsets in metres, chosen so labels of neighbouring frames do not collide:
# forearm and wrist sit about 30 mm apart.
shown = {"base_link": (0.02, 0, 0.0), "shoulder_link": (-0.075, 0, 0.0),
         "upper_arm_link": (-0.085, 0, 0.0), "forearm_link": (-0.075, 0, 0.012),
         "wrist_link": (0.02, 0, 0.02), "tool_mount_link": (0.03, 0, 0.02),
         "tool0": (0.03, 0, -0.005), "tattoo_tcp": (0.02, 0, -0.03)}
for name, off in shown.items():
    T = frames[name]
    L = 0.045 if name != "tattoo_tcp" else 0.03
    for k in range(3):
        a, b = T[:3, 3], T[:3, 3] + L * T[:3, k]
        ax.plot(*zip(a, b), color=AXC[k], lw=2.2, zorder=10)
    ax.scatter(*T[:3, 3], color=INK2, s=10, zorder=11)
    ax.text(*(T[:3, 3] + off), name.replace("_link", ""), fontsize=8, color=INK2, zorder=12)
pts = np.vstack([v for _, v, _, _ in nb.posed(parts, chain, q_draw)])
nb.equal_3d(ax, pts)
ax.view_init(elev=22, azim=-58)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_zlabel("z (m)")
ax.set_title("Every frame of the chain, at a pose from the drawing", loc="left")
nb.show(fig, export="19_frames.png")

# %% [markdown]
# ## 4 · The product of exponentials
#
# The same forward kinematics, written the way modern robotics texts do. At the
# zero pose each joint is a **screw axis** $\mathcal S_i = (\omega_i, v_i)$ in the
# base frame: $\omega_i$ its direction and $v_i = -\omega_i \times p_i$, with $p_i$
# any point on it. With $M$ the tip's pose at zero,
#
# $$T(q) = e^{[\mathcal S_1] q_1}\, e^{[\mathcal S_2] q_2} \cdots e^{[\mathcal S_5] q_5}\, M .$$
#
# The exponential of a twist has a closed form:
#
# $$e^{[\mathcal S]\theta} = \begin{bmatrix} e^{[\omega]\theta} &
#   \left(I\theta + (1-\cos\theta)[\omega] + (\theta - \sin\theta)[\omega]^2\right) v \\
#   0 & 1 \end{bmatrix}.$$

# %%
zero = dict(chain.frames(np.zeros(5)))
S = np.zeros((6, 5))
for j, s in enumerate(chain.actuated):
    Tj = zero[s["child"]]
    w = Tj[:3, :3] @ (s["axis"] / np.linalg.norm(s["axis"]))
    S[:3, j] = w
    S[3:, j] = -np.cross(w, Tj[:3, 3])
M = chain.fk(np.zeros(5))

print("Screw axes, one column per joint (rows: ωx ωy ωz vx vy vz):")
nb.display(nb.matrix(S))
print("M, the tip at the zero pose:")
nb.display(nb.matrix(M))


def exp_twist(Sj, t):
    w, v = Sj[:3], Sj[3:]
    W = skew_np(w)
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(t) * W + (1 - np.cos(t)) * W @ W
    T[:3, 3] = (np.eye(3) * t + (1 - np.cos(t)) * W + (t - np.sin(t)) * W @ W) @ v
    return T


def twist_hat(Sj):
    X = np.zeros((4, 4))
    X[:3, :3] = skew_np(Sj[:3])
    X[:3, 3] = Sj[3:]
    return X


def fk_poe(qq):
    T = np.eye(4)
    for j in range(5):
        T = T @ exp_twist(S[:, j], qq[j])
    return T @ M


worst_exp = worst_fk = 0.0
for qq in rng.uniform(LOWER, UPPER, (200, 5)):
    for j in range(5):
        worst_exp = max(worst_exp, np.abs(exp_twist(S[:, j], qq[j])
                                          - expm(twist_hat(S[:, j]) * qq[j])).max())
    worst_fk = max(worst_fk, np.abs(fk_poe(qq) - chain.fk(qq)).max())
print(f"closed-form exponential against expm: worst {worst_exp:.1e}")
print(f"product of exponentials against the URDF chain, 200 poses: worst {worst_fk:.1e}")
assert worst_exp < 1e-12 and worst_fk < 1e-12

# %% [markdown]
# ## 5 · Velocity kinematics and Jacobians
#
# The **space Jacobian** stacks each joint's screw axis, carried to the current
# configuration by the adjoint of the transforms before it:
#
# $$J_s(q) = \begin{bmatrix} \mathcal S_1 & \mathrm{Ad}_{e^{[\mathcal S_1]q_1}}\mathcal S_2 &
#   \cdots \end{bmatrix}, \qquad
#   \mathrm{Ad}_T = \begin{bmatrix} R & 0 \\ [p]R & R \end{bmatrix}.$$
#
# Its linear part is the velocity of the body point passing through the base
# origin. The **geometric Jacobian** the repository uses gives the velocity of
# the needle tip instead; the two are related by $v_{\text{tip}} = v_s + \omega
# \times p_{\text{tip}}$. Both are checked, and the tip's linear rows are checked
# once more against finite differences of the forward kinematics.

# %%
def Ad(T):
    R, p = T[:3, :3], T[:3, 3]
    A = np.zeros((6, 6))
    A[:3, :3] = R
    A[3:, 3:] = R
    A[3:, :3] = skew_np(p) @ R
    return A


def space_jacobian(qq):
    J = np.zeros((6, 5))
    T = np.eye(4)
    for j in range(5):
        J[:, j] = Ad(T) @ S[:, j]
        T = T @ exp_twist(S[:, j], qq[j])
    return J


Jg = chain.jacobian(q_draw)            # rows: linear velocity of the tip, then angular
Js = space_jacobian(q_draw)
p = chain.tcp(q_draw)
assert np.allclose(Jg[3:], Js[:3], atol=1e-12)
assert np.allclose(Jg[:3], Js[3:] - skew_np(p) @ Js[:3], atol=1e-12)

h = 1e-7
J_fd = np.column_stack([(chain.tcp(q_draw + h * e) - chain.tcp(q_draw - h * e)) / (2 * h)
                        for e in np.eye(5)])
print(f"tip Jacobian against central differences: worst {np.abs(J_fd - Jg[:3]).max():.1e}")
assert np.abs(J_fd - Jg[:3]).max() < 1e-8

print("Geometric Jacobian at the tip, at a pose from the drawing (6 × 5):")
nb.display(nb.matrix(Jg))

# %% [markdown]
# ### The task Jacobian: why five axes are enough
#
# A full pose is six numbers, and the arm has five joints. But tattooing does not
# ask for a full pose: it asks for the tip at a point (three numbers) and the needle
# along a direction (two numbers — a direction is a point on a sphere). Spinning the
# needle about its own axis changes nothing, because the needle is round. So the
# task is five constraints against five joints, and the Jacobian is **square**:
#
# $$J_{\text{task}} = \begin{bmatrix} J_v \\ u^\top [\,\omega \times z\,] \\
#   v^\top [\,\omega \times z\,] \end{bmatrix} \in \mathbb R^{5 \times 5},$$
#
# with $u, v$ spanning the plane normal to the needle axis $z$. The two direction
# rows are checked against finite differences of $z$.

# %%
Jt = chain.task_jacobian(q_draw)
z = chain.tool_axis(q_draw)
a = np.array([1.0, 0.0, 0.0]) if abs(z[0]) <= 0.9 else np.array([0.0, 1.0, 0.0])
u = np.cross(z, a)
u /= np.linalg.norm(u)
v = np.cross(z, u)
dz = np.column_stack([(chain.tool_axis(q_draw + h * e) - chain.tool_axis(q_draw - h * e)) / (2 * h)
                      for e in np.eye(5)])
assert np.abs(Jt[3] - u @ dz).max() < 1e-8 and np.abs(Jt[4] - v @ dz).max() < 1e-8
print("Task Jacobian (5 × 5), determinant", f"{np.linalg.det(Jt):.3e}:")
nb.display(nb.matrix(Jt))

# %% [markdown]
# ## 6 · Singularities and manipulability
#
# The singular values $\sigma_1 \ge \dots \ge \sigma_5$ of the task Jacobian say how
# much tip motion a unit of joint motion buys in each direction. The smallest,
# $\sigma_5$, is the one that matters: as it approaches zero the arm approaches a
# singularity, and the joint motion needed to move the tip in that direction grows
# without bound. Along the whole drawing:

# %%
Q = traj["q"][::10]
t = traj["t"][::10]
sv = np.array([np.linalg.svd(chain.task_jacobian(qq), compute_uv=False) for qq in Q])

import json  # noqa: E402
study = json.loads((nb.REPO / "docs" / "data" / "kinematics_study.json").read_text())["path"]
print(f"σ5 along the drawing: {sv[:, 4].min():.4f} … {sv[:, 4].max():.4f} "
      f"(kinematics_study.json: {study['sigma5_min']:.4f} … {study['sigma5_max']:.4f})")
print(f"condition number: {(sv[:, 0] / sv[:, 4]).min():.1f} … {(sv[:, 0] / sv[:, 4]).max():.1f}")
assert abs(sv[:, 4].min() - study["sigma5_min"]) < 5e-4

# %% [markdown]
# The **manipulability ellipsoid** makes it visible. For the tip's linear velocity,
# the image of the unit ball of joint rates is an ellipsoid with semi-axes along the
# left singular vectors of $J_v$ and lengths $\sigma_i$: long where the arm moves the
# tip easily, thin where it struggles. Measured below: the long axis stays in the
# vertical plane through the base, so reaching in, out, up and down is easy and
# moving sideways — which only `joint_1` provides — is not.

# %%
def ellipsoid(Jv, centre, scale, n=18):
    U, s, _ = np.linalg.svd(Jv)
    a, b = np.meshgrid(np.linspace(0, 2 * np.pi, n), np.linspace(0, np.pi, n // 2))
    sphere = np.stack([np.cos(a) * np.sin(b), np.sin(a) * np.sin(b), np.cos(b)], -1)
    return centre + scale * (sphere * s[:3]) @ U.T


fig = plt.figure(figsize=(11.5, 4.8))
ax1 = fig.add_subplot(1, 2, 1)
for k in range(5):
    ax1.plot(t, sv[:, k], color=C[k], lw=1.2 if k < 4 else 1.8, label=f"σ{k + 1}")
ax1.set_yscale("log")
ax1.set_xlabel("time along the drawing (s)")
ax1.set_ylabel("singular value")
ax1.set_title("Singular values of the task Jacobian", loc="left")
ax1.grid(True, alpha=0.9)
ax1.legend(ncol=5, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.16))

# The ellipsoids are drawn over the panel alone: with the whole arm in view they
# are a few pixels across. Scaled so the largest semi-axis spans 35 mm.
ax2 = fig.add_subplot(1, 2, 2, projection="3d")
tip = traj["tcp"]
mk = traj["marked"]
ax2.plot(tip[mk, 0], tip[mk, 1], tip[mk, 2], ",", color=C[6], alpha=0.7)
ax2.plot([0.11, 0.31, 0.31, 0.11, 0.11], [-0.07, -0.07, 0.07, 0.07, -0.07], 0.005,
         color=C[1], lw=1.0)
samples = traj["q"][np.linspace(0, len(traj["q"]) - 1, 9).astype(int)]
largest = max(np.linalg.svd(chain.jacobian(qq)[:3], compute_uv=False)[0] for qq in samples)
for qq in samples:
    E = ellipsoid(chain.jacobian(qq)[:3], chain.tcp(qq), 0.035 / largest)
    ax2.plot_surface(E[..., 0], E[..., 1], E[..., 2], color=C[0], alpha=0.45, linewidth=0)
ax2.set_xlim(0.10, 0.32)
ax2.set_ylim(-0.11, 0.11)
ax2.set_zlim(-0.10, 0.12)
ax2.set_box_aspect((1, 1, 1))
ax2.view_init(elev=28, azim=-62)
ax2.set_xticks([0.12, 0.2, 0.28])
ax2.set_yticks([-0.08, 0, 0.08])
ax2.set_zticks([-0.08, 0, 0.08])
ax2.set_xlabel("x (m)")
ax2.set_ylabel("y (m)")
ax2.set_title("Tip manipulability ellipsoids at nine points of the drawing", loc="left")
nb.show(fig, export="20_manipulability.png")

for qq in samples:
    U = np.linalg.svd(chain.jacobian(qq)[:3])[0]
    p_ = chain.tcp(qq)
    radial = np.array([p_[0], p_[1], 0.0]) / np.hypot(p_[0], p_[1])
    sideways = np.cross([0.0, 0.0, 1.0], radial)
    assert abs(U[:, 0] @ sideways) < np.sin(np.radians(6))
print("the long axis of every ellipsoid lies within 6° of the vertical plane through the base")

# %% [markdown]
# ## 7 · Workspace
#
# Sampling the joint box uniformly and plotting where the tip lands. This bounds
# the envelope; uniform in joint space is not uniform in Cartesian space, so the
# density shows where the arm has the most configurations available, not the
# exact reachable volume. The panel it draws on sits well inside.

# %%
Qs = rng.uniform(LOWER, UPPER, (20000, 5))
P = np.array([chain.tcp(qq) for qq in Qs])
fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.4))
a1.scatter(P[:, 0], P[:, 2], s=0.3, color=C[0], alpha=0.25)
a1.set_xlabel("x (m)")
a1.set_ylabel("z (m)")
a1.set_title("Side view", loc="left")
a2.scatter(P[:, 0], P[:, 1], s=0.3, color=C[0], alpha=0.25)
a2.add_patch(plt.Rectangle((0.21 - 0.10, -0.07), 0.20, 0.14, fill=False, ec=C[1], lw=1.6))
a2.text(0.21, 0.08, "panel", color=C[1], ha="center", fontsize=8)
a2.set_xlabel("x (m)")
a2.set_ylabel("y (m)")
a2.set_title("Top view", loc="left")
for a in (a1, a2):
    a.set_aspect("equal")
    a.grid(True, alpha=0.9)
nb.show(fig)

try:
    import plotly.graph_objects as go
    fig3 = go.Figure(go.Scatter3d(x=P[::4, 0], y=P[::4, 1], z=P[::4, 2], mode="markers",
                                  marker=dict(size=1.2, color=P[::4, 2], colorscale="Blues")))
    fig3.update_layout(title="Workspace (drag to rotate)", scene_aspectmode="data",
                       height=520, margin=dict(l=0, r=0, t=40, b=0))
    nb.show(fig3)
except ImportError:
    print("plotly is not installed; the interactive view is skipped")

# %% [markdown]
# ## 8 · Inverse kinematics
#
# Given a tip position $p^\star$ and needle direction $z^\star$, find $q$. There is
# no closed form worth having here, so it is solved iteratively by **damped least
# squares** (Levenberg–Marquardt) on the task Jacobian:
#
# $$\Delta q = J^\top \left(J J^\top + \lambda I\right)^{-1} e, \qquad \lambda = 10^{-3},$$
#
# with $e$ the five task errors. The damping keeps the step bounded near
# ill-conditioned poses. It is written out here in full, then required to give
# exactly what the repository's `chain.ik` gives.

# %%
def task_error(qq, p_des, z_des):
    T = chain.fk(qq)
    z = T[:3, 2]
    a = np.array([1.0, 0.0, 0.0]) if abs(z[0]) <= 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(z, a)
    u /= np.linalg.norm(u)
    v = np.cross(z, u)
    e_z = z_des - z
    return np.concatenate([p_des - T[:3, 3], [u @ e_z, v @ e_z]])


def ik_dls(p_des, z_des, q0, lam=1e-3, tol=1e-6, iters=200):
    qq, history = np.array(q0, float), []
    for _ in range(iters):
        e = task_error(qq, p_des, z_des)
        history.append((qq.copy(), np.linalg.norm(e)))
        if np.linalg.norm(e) < tol:
            break
        J = chain.task_jacobian(qq)
        qq = np.clip(qq + J.T @ np.linalg.solve(J @ J.T + lam * np.eye(5), e), LOWER, UPPER)
    return qq, history


down = np.array([0.0, 0.0, -1.0])
q_home = np.array([0.0, 0.6, -0.9, 0.0, -1.2])       # the start analysis.solve_path uses
targets = [np.array([0.21 + dx, dy, 0.005]) for dx, dy in
           [(0.0, 0.0), (0.07, 0.05), (-0.07, -0.05), (0.07, -0.06), (-0.08, 0.06)]]
fig, ax = plt.subplots(figsize=(7, 3.6))
for k, target in enumerate(targets):
    q_sol, hist = ik_dls(target, down, q_home)
    q_ref, ok, _ = chain.ik(target, down, q_home)
    assert ok and np.allclose(q_sol, q_ref, atol=1e-12)
    ax.semilogy([h_[1] for h_ in hist], "-o", ms=3, color=C[k],
                label=f"({(target[0] - 0.21) * 1000:+.0f}, {target[1] * 1000:+.0f}) mm")
ax.axhline(1e-6, color=MUTED, lw=0.8, ls="--")
ax.text(0.2, 1.4e-6, "tolerance", color=MUTED, fontsize=8)
ax.set_xlabel("iteration")
ax.set_ylabel("task error ‖e‖")
ax.set_title("Damped least squares from the start pose, to five points on the panel",
             loc="left")
ax.legend(title="offset from panel centre", fontsize=8, title_fontsize=8)
ax.grid(True, alpha=0.9)
nb.show(fig, export="22_ik_convergence.png")
print("every solution equals chain.ik's to 1e-12")

# %% [markdown]
# Two things show in that plot. The error *rises* on the first step: from the start
# pose the full Gauss–Newton step overshoots, and with $\lambda$ this small the
# damping does not hold it back. After that it falls by a roughly constant factor
# per iteration, a straight line on a log axis, reaching the $10^{-6}$ tolerance in
# nine to twelve steps. Along the drawing each solve starts from the previous
# point's solution, a warm start a fraction of a millimetre away, and takes about a
# third as many — measured here over 400 consecutive points of the logo:

# %%
steps = [len(ik_dls(traj["tcp"][j], down, traj["q"][j - 1])[1]) - 1 for j in range(1, 401)]
print(f"warm-started iterations per point: mean {np.mean(steps):.2f}, most {max(steps)}")
assert np.mean(steps) < 5          # measured 2.93 when this was written

# %% [markdown]
# Watching the iterations, side view: the arm unfolding from its start pose onto the
# first target.

# %%
q_sol, hist = ik_dls(targets[1], down, q_home)
skeleton_links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
                  "wrist_link", "tool_mount_link", "tool0", "tattoo_tcp"]


def skeleton(qq):
    f = dict(chain.frames(qq))
    return np.array([f[n][:3, 3] for n in skeleton_links])


fig, ax = plt.subplots(figsize=(6, 4.2))
ax.plot([0.11, 0.31], [0.005, 0.005], color=C[1], lw=3)
line, = ax.plot([], [], "-o", color=INK2, lw=2.2, ms=4)
tgt = ax.plot(targets[1][0], targets[1][2], "x", color=C[7], ms=9)
label = ax.text(0.02, 0.33, "", fontsize=9)
ax.set_xlim(-0.05, 0.35)
ax.set_ylim(-0.02, 0.36)
ax.set_aspect("equal")
ax.set_xlabel("x (m)")
ax.set_ylabel("z (m)")
ax.grid(True, alpha=0.9)


def frame_ik(k):
    pts = skeleton(hist[k][0])
    line.set_data(pts[:, 0], pts[:, 2])
    label.set_text(f"iteration {k}   ‖e‖ = {hist[k][1]:.1e}")
    return line, label


nb.animate(animation.FuncAnimation(fig, frame_ik, frames=len(hist), interval=400), fps=3)

# %% [markdown]
# ## 9 · The arm drawing, in 2D and 3D
#
# The trajectory the simulation runs — every joint angle for every point of the
# official ROS logo, solved offline by the same inverse kinematics — replayed
# here. First in 2D: the arm from the side, and the panel from above as the ink
# goes down.

# %%
n_frames = 72
idx = np.linspace(0, len(traj["q"]) - 1, n_frames).astype(int)
fig, (s1, s2) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={"width_ratios": [1, 1.25]})
s1.plot([0.11, 0.31], [0.005, 0.005], color=C[1], lw=3)
arm_line, = s1.plot([], [], "-o", color=INK2, lw=2.2, ms=4)
s1.set_xlim(-0.05, 0.35)
s1.set_ylim(-0.02, 0.36)
s1.set_aspect("equal")
s1.set_title("Side view", loc="left")
s2.add_patch(plt.Rectangle((0.11, -0.07), 0.20, 0.14, color="#efe4df"))
ink, = s2.plot([], [], ",", color=C[6])
pen, = s2.plot([], [], "o", color=C[1], ms=5)
s2.set_xlim(0.10, 0.32)
s2.set_ylim(-0.08, 0.08)
s2.set_aspect("equal")
s2.set_title("The panel, from above", loc="left")
clock = s2.text(0.11, 0.072, "", fontsize=8, color=INK2)


def frame_2d(k):
    j = idx[k]
    pts = skeleton(traj["q"][j])
    arm_line.set_data(pts[:, 0], pts[:, 2])
    m = traj["marked"][:j + 1]
    ink.set_data(traj["tcp"][:j + 1][m, 0], traj["tcp"][:j + 1][m, 1])
    pen.set_data([traj["tcp"][j, 0]], [traj["tcp"][j, 1]])
    clock.set_text(f"t = {traj['t'][j]:5.0f} s of {traj['t'][-1]:.0f} s")
    return arm_line, ink, pen, clock


nb.animate(animation.FuncAnimation(fig, frame_2d, frames=n_frames, interval=100), fps=12)

# %% [markdown]
# And in 3D, with the arm's real meshes. For animation they are simplified by
# vertex clustering on a 4 mm grid — from about fifty thousand triangles to a few
# thousand — because redrawing the full meshes every frame is slow; the static
# frame figure above uses them unsimplified.

# %%
low = nb.visual_parts(cell=0.004)
print("triangles, full / simplified:", sum(len(f) for *_, f, _ in parts),
      "/", sum(len(f) for *_, f, _ in low))
fig = plt.figure(figsize=(7.2, 5.6))
ax = fig.add_subplot(projection="3d")
fig.subplots_adjust(left=0, right=1, bottom=0, top=0.94)
panel = np.array([[0.11, -0.07, 0.005], [0.31, -0.07, 0.005], [0.31, 0.07, 0.005],
                  [0.11, 0.07, 0.005]])
bounds = np.vstack([panel] + [nb.posed(low, chain, traj["q"][j])[k][1]
                              for j in idx[::15] for k in range(len(low))])


def frame_3d(k):
    j = idx[k]
    ax.cla()
    ax.plot([0.11, 0.31, 0.31, 0.11, 0.11], [-0.07, -0.07, 0.07, 0.07, -0.07], 0.005,
            color=C[1], lw=1.2)
    nb.draw_arm(ax, nb.posed(low, chain, traj["q"][j]), alpha=1.0)
    m = traj["marked"][:j + 1]
    ax.plot(traj["tcp"][:j + 1][m, 0], traj["tcp"][:j + 1][m, 1], traj["tcp"][:j + 1][m, 2],
            ",", color=C[6])
    nb.equal_3d(ax, bounds, pad=0.0, zoom=1.3)
    # The letters run along +x and stand along +y, so the logo reads correctly
    # from the -y side; the camera sweeps around that side, the arm in profile.
    ax.view_init(elev=30, azim=-118 + 36 * k / n_frames)
    ax.set_axis_off()
    ax.set_title(f"TATTOTRONIX drawing the ROS logo    t = {traj['t'][j]:4.0f} s",
                 loc="left", fontsize=10)
    return []


nb.animate(animation.FuncAnimation(fig, frame_3d, frames=n_frames, interval=100),
           export="21_arm_3d.gif", fps=12)

# %% [markdown]
# Finally an interactive view: the arm at the same pose with its **full** meshes,
# and the logo it draws. Drag to rotate, scroll to zoom.

# %%
try:
    import plotly.graph_objects as go
    shade = {"tattotronix_shell": "#c9c8c3", "tattotronix_steel": "#8a8984",
             "tattotronix_ink": "#2b2b2b"}
    data = [go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], i=f[:, 0], j=f[:, 1], k=f[:, 2],
                      color=shade.get(mat, "#c9c8c3"), flatshading=True, name=link)
            for link, v, f, mat in nb.posed(parts, chain, q_draw)]
    mk = traj["marked"]
    data.append(go.Scatter3d(x=tip[mk, 0], y=tip[mk, 1], z=tip[mk, 2], mode="markers",
                             marker=dict(size=1, color=C[6]), name="ink"))
    fig3 = go.Figure(data)
    fig3.update_layout(scene_aspectmode="data", height=620, showlegend=False,
                       margin=dict(l=0, r=0, t=30, b=0), title="The arm and its drawing")
    nb.show(fig3)
except ImportError:
    print("plotly is not installed; the interactive view is skipped")
