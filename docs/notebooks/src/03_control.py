# %% [markdown]
# # Control of the TATTOTRONIX arm
#
# Dynamics says what torque the arm needs. Control is how each servo finds it:
# read where the joint is, compare it with where it should be, and push. This
# notebook derives the joint controller the repository's studies use, from where
# its poles are placed to how fast it has to run, and checks each step against the
# data those studies wrote. The `assert` lines run on every build of the repository.
#
# The loop here is the one each servo has to close on the real arm, studied
# against the full non-linear dynamics. The running Gazebo simulation does not
# execute it: there, each joint is driven to its setpoint directly. See
# [control](../mathematical-model/control.md#2-control-structure).
#
# **Contents.** 1 · One joint, three poles — 2 · Where the overshoot comes from —
# 3 · Following a moving line — 4 · Five joints are not five loops — 5 · The
# sampled loop, and the rate ceiling — 6 · The non-linear arm past the ceiling

# %% [setup]

# %%
import json
from itertools import permutations

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.colors import LinearSegmentedColormap
from scipy import signal

import analysis               # the repository's tuning and closed-loop simulation
import style
from style import BLUES, C, INK2, MUTED

style.apply(**{"figure.dpi": 110})
np.set_printoptions(precision=4, suppress=True)

chain, model = nb.arm()
NAMES = [s["name"] for s in chain.actuated]
DATA = nb.REPO / "docs" / "data"
summary = json.loads((DATA / "summary.json").read_text())
rate_study = json.loads((DATA / "rate_study.json").read_text())
q_ref = np.array(summary["q_ref_tune"])      # the pose the published gains are tuned at
WN = summary["tune_wn"]                      # and the bandwidth they are tuned for, rad/s

# %% [markdown]
# ## 1 · One joint, three poles
#
# The model's own gravity torque $G(q)$ is added to every command, so what a
# joint's feedback has left to move is its inertia, $J\ddot q = \tau$: a double
# integrator. The controller the studies run is a PID whose derivative acts on the
# measured velocity,
#
# $$\tau = K_p\,e + K_i\!\int\! e\,dt - K_d\,\dot q, \qquad e = q_r - q .$$
#
# Closing the loop gives the characteristic polynomial
# $J s^3 + K_d s^2 + K_p s + K_i$, whose roots are the closed loop's poles, so
# choosing the poles chooses the gains. Putting all three at $-\omega_n$ leaves a
# single number to decide, the bandwidth $\omega_n$:

# %%
s = sp.symbols("s")
J, w = sp.symbols("J omega_n", positive=True)
Kp, Kd, Ki = sp.symbols("K_p K_d K_i", positive=True)
closed = J * s**3 + Kd * s**2 + Kp * s + Ki
gains = sp.solve(sp.Poly(closed - J * (s + w)**3, s).coeffs(), [Kp, Kd, Ki], dict=True)[0]
for k in (Kp, Kd, Ki):
    nb.display(sp.Eq(k, gains[k]))

# %% [markdown]
# $J$ is each joint's **effective** inertia, $1/(M^{-1})_{ii}$: what it has to
# accelerate when the other joints are free to move, which nothing stops them
# doing. Section 4 shows what that choice accounts for, and what it cannot. At the
# pose the published gains are tuned at, with $\omega_n$ = 20 rad/s:

# %%
M_ref = model.inertia(q_ref)
J_eff = 1 / np.diag(np.linalg.inv(M_ref))
Kp_n, Ki_n, Kd_n = analysis.tune(model, q_ref)
for sym, theirs in ((Kp, Kp_n), (Kd, Kd_n), (Ki, Ki_n)):
    mine = sp.lambdify((J, w), gains[sym])(J_eff, WN)
    assert np.allclose(mine, theirs, rtol=1e-12)
assert np.allclose(Kp_n.round(4), summary["Kp"]) and np.allclose(Ki_n.round(4), summary["Ki"])
assert np.allclose(Kd_n.round(5), summary["Kd"])
nb.table(["joint", "J_eff (kg·m²)", "K_p (N·m/rad)", "K_i (N·m/(rad·s))", "K_d (N·m·s/rad)"],
         [[f"`{n}`", f"{J_eff[i]:.3e}", f"{Kp_n[i]:.4g}", f"{Ki_n[i]:.4g}", f"{Kd_n[i]:.4g}"]
          for i, n in enumerate(NAMES)])
print("the gains analysis.tune() computes, and summary.json publishes")

# %% [markdown]
# ## 2 · Where the overshoot comes from
#
# Placing the poles fixes the closed loop's denominator. Where the reference
# enters the controller fixes its numerator, whose roots — the **zeros** — shape
# the response as much as the poles do. Three ways to give the same PID its
# reference, all with the same three poles:
#
# - **PI-D**: the derivative on the measurement, as above.
# - **PID**: the derivative on the error, $K_d\,\dot e$. That is exactly what
#   feeding the reference velocity forward, $K_d\,\dot q_r$, adds to PI-D, so the
#   studies' velocity feedforward behaves like this row.
# - **I-PD**: only the integral sees the reference; the proportional and derivative
#   terms act on the measurement alone.
#
# Measured in units of $1/\omega_n$, time makes the gains drop out, and each closed
# loop $T(s) = Q(s)/Q_r(s)$ is a fixed function:

# %%
den = closed.subs(gains)
numerators = {"PI-D": Kp * s + Ki, "PID": Kd * s**2 + Kp * s + Ki, "I-PD": Ki}
T = {k: sp.factor(sp.cancel((num.subs(gains) / den).subs(w, 1)))
     for k, num in numerators.items()}
nb.table(["structure", "T(s), time in units of 1/ωn"],
         [[k, f"${sp.latex(v)}$"] for k, v in T.items()])

# %% [markdown]
# PI-D and PID have zeros; I-PD has none. The step response of each is the
# inverse Laplace transform of $T(s)/s$, and its peak is where its derivative, the
# impulse response, first returns to zero:

# %%
t = sp.symbols("t", positive=True)
step = {k: sp.simplify(sp.inverse_laplace_transform(v / s, s, t)) for k, v in T.items()}
overshoot, rows = {}, []
for k, y in step.items():
    turns = [r for r in sp.solve(sp.diff(y, t), t) if r.is_positive]
    if turns:
        t_pk = min(turns, key=float)
        overshoot[k] = (t_pk, sp.simplify(y.subs(t, t_pk) - 1))
        peak = (f"${sp.latex(overshoot[k][1])}$ = {float(overshoot[k][1]) * 100:.2f}%, "
                f"at $t = {sp.latex(t_pk)}$")
    else:
        impulse = sp.simplify(sp.diff(y, t))
        peak = f"none: the impulse response ${sp.latex(impulse)}$ is never negative"
    rows.append([k, f"${sp.latex(sp.collect(sp.expand(y), sp.exp(-t)))}$", peak])
nb.table(["structure", "step response y(t)", "overshoot"], rows)

tn = np.linspace(0, 12, 2401)
for k, v in T.items():
    num, dn = (np.array(sp.Poly(e, s).all_coeffs(), float) for e in sp.fraction(v))
    _, y_scipy = signal.step((num, dn), T=tn)
    assert np.abs(sp.lambdify(t, step[k])(tn[1:]) - y_scipy[1:]).max() < 1e-6
pid_pct = float(overshoot["PI-D"][1]) * 100
assert abs(pid_pct - summary["step_overshoot_single_axis_pct"]) < 1e-6
assert sp.simplify(overshoot["PI-D"][1] - 5 * sp.exp(-3)) == 0
assert round(float(overshoot["PID"][1]) * 100) == 21 and "I-PD" not in overshoot
print("closed forms agree with scipy's step response; PI-D's 5/e³ is the figure "
      "analysis.py publishes")

# %% [markdown]
# PI-D, the structure the studies run without feedforward, overshoots by exactly
# $5/e^3$ — 24.9% — at $t = 3/\omega_n$, whatever the joint's inertia. Its zero
# sits at $-\omega_n/3$, slower than the poles, and that is what drives it past the
# target. Moving the derivative onto the error adds a second zero and lowers the
# overshoot to $(\sqrt3 - 1)\,e^{\sqrt3 - 3}$, 20.6%. I-PD removes the zeros and the
# overshoot with them, and the next section shows what it costs.

# %% [markdown]
# ## 3 · Following a moving line
#
# A drawing is not a step. The reference moves, and what matters is how far behind
# it the joint runs. For a reference that starts moving at speed $v$,
# $Q_r(s) = v/s^2$, and the error is $E(s) = (1 - T(s))\,v/s^2$. In units of
# $v/\omega_n$:

# %%
ramp = {k: sp.simplify(sp.inverse_laplace_transform(sp.cancel((1 - v) / s**2), s, t))
        for k, v in T.items()}
lag, rows = {}, []
for k, e in ramp.items():
    turns = [r for r in sp.solve(sp.diff(e, t), t) if r.is_positive]
    if turns:
        t_pk = min(turns, key=float)
        lag[k] = (t_pk, sp.simplify(e.subs(t, t_pk)))
        worst = (f"${sp.latex(lag[k][1])}$ = {float(lag[k][1]):.3f}, "
                 f"at $t = {sp.latex(t_pk)}$")
    else:
        worst = f"tends to {sp.limit(e, t, sp.oo)} and stays there"
    rows.append([k, f"${sp.latex(e)}$", worst])
nb.table(["structure", "following error e(t)", "largest"], rows)

for k, v in T.items():
    num, dn = (np.array(sp.Poly(e, s).all_coeffs(), float) for e in sp.fraction(v))
    _, y_ramp, _ = signal.lsim((num, dn), U=tn, T=tn)
    assert np.abs(sp.lambdify(t, ramp[k])(tn[1:]) - (tn - y_ramp)[1:]).max() < 1e-6
assert sp.simplify(lag["PI-D"][0] - (1 + sp.sqrt(5)) / 2) == 0    # the golden ratio
assert round(float(lag["PI-D"][1]), 2) == 0.84           # control.md, section 3
assert round(float(lag["PID"][1]), 2) == 0.23 and lag["PID"][0] < lag["PI-D"][0]
assert sp.limit(ramp["I-PD"], t, sp.oo) == 3
print("closed forms agree with scipy's simulation of each loop")

# %% [markdown]
# The PI-D error peaks at $0.84\,v/\omega_n$, at the golden ratio,
# $t = \varphi/\omega_n$, and the integral then walks it back out. Feeding the
# reference velocity forward cuts the peak to $0.23\,v/\omega_n$, and it arrives
# sooner. I-PD, which never overshoots a step, pays for it here: it settles into a
# lag of $3\,v/\omega_n$ and keeps it for as long as the line keeps moving. A
# drawing is almost all moving line, which is why the studies feed the velocity
# forward rather than take the overshoot out this way. Feeding the acceleration
# forward too — the
# computed-torque term $M\ddot q_r$ — makes the numerator equal the denominator:
# $T(s) = 1$, and no lag at all, for as long as the model is exact.

# %%
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.1))
colours = {"PI-D": C[0], "PID": C[1], "I-PD": C[2]}
labels = {"PI-D": "PI-D, derivative on the measurement",
          "PID": "PID, or PI-D with velocity feedforward",
          "I-PD": "I-PD, only the integral sees the reference"}
for k in T:
    a1.plot(tn, sp.lambdify(t, step[k])(tn), color=colours[k], label=labels[k])
    a2.plot(tn, sp.lambdify(t, ramp[k])(tn), color=colours[k], label=labels[k])
a1.axhline(1, color=MUTED, lw=0.8)
for k, (t_pk, o) in overshoot.items():
    a1.plot(float(t_pk), 1 + float(o), "o", color=colours[k], ms=6)
    a1.annotate(f"{float(o) * 100:.1f}%", (float(t_pk), 1 + float(o)),
                xytext=(6, 5) if k == "PI-D" else (-42, 4), textcoords="offset points",
                fontsize=9, color=INK2)
for k, (t_pk, e) in lag.items():
    a2.plot(float(t_pk), float(e), "o", color=colours[k], ms=6)
    a2.annotate(f"{float(e):.2f} v/ωn", (float(t_pk), float(e)), xytext=(6, 6),
                textcoords="offset points", fontsize=9, color=INK2)
a2.annotate("3 v/ωn, for good", (12, 3), xytext=(-4, -14), textcoords="offset points",
            ha="right", fontsize=9, color=INK2)
a1.set_xlabel("time × ωn")
a1.set_ylabel("joint position / step")
a1.set_title("A step: the same three poles, three responses", loc="left")
a2.set_xlabel("time × ωn")
a2.set_ylabel("following error / (v / ωn)")
a2.set_title("A ramp: how far the joint runs behind", loc="left")
for ax in (a1, a2):
    ax.grid(True, alpha=0.9)
a1.legend(fontsize=8, loc="lower right")
nb.show(fig, export="26_loop_structures.png")

# %% [markdown]
# ## 4 · Five joints are not five loops
#
# Each joint's gains were chosen against its own effective inertia, as if it were
# alone. It is not. With gravity cancelled the arm obeys $M(q)\,\ddot q = \tau$, and
# $M$ is full. But every gain is a multiple of one diagonal matrix,
# $\mathcal J = \operatorname{diag}(J_{\text{eff}})$:
#
# $$K_p = 3\omega_n^2\,\mathcal J, \qquad K_d = 3\omega_n\,\mathcal J, \qquad
#   K_i = \omega_n^3\,\mathcal J .$$
#
# Differentiating the closed loop once to clear the integral, with $\tilde q$ the
# deviation from a fixed reference,
#
# $$\dddot{\tilde q} + M^{-1}\mathcal J\,\bigl(3\omega_n\ddot{\tilde q} +
#   3\omega_n^2\dot{\tilde q} + \omega_n^3\tilde q\bigr) = 0 .$$
#
# $M^{-1}\mathcal J$ is similar to the symmetric positive definite matrix
# $\mathcal J^{1/2} M^{-1} \mathcal J^{1/2}$, so its eigenvalues $\lambda_i$ are real
# and positive and its eigenvectors form a basis. Along each eigenvector the loop is
# a single axis again, with every gain scaled by $\lambda_i$:
#
# $$s^3 + \lambda_i\bigl(3\omega_n s^2 + 3\omega_n^2 s + \omega_n^3\bigr) = 0 .$$
#
# A mode with $\lambda = 1$ is the loop that was designed. And because
# $\operatorname{tr}(M^{-1}\mathcal J) = \sum_i (M^{-1})_{ii}\,J_{\text{eff},i} = 5$,
# the $\lambda_i$ average exactly one: tuning against the effective inertia is right
# on average, and only on average.

# %%
M_inv = np.linalg.inv(M_ref)
lam = np.sort(np.linalg.eigvals(M_inv @ np.diag(J_eff)).real)[::-1]
print("λ, the share of its designed gain each mode receives:", np.round(lam, 3),
      f"  (sum {lam.sum():.12f})")
assert abs(lam.sum() - 5) < 1e-12
assert np.allclose(lam, summary["modal_gain"], rtol=1e-9)

# The repository builds the whole 15-state loop, with no modes in it.
poles = np.linalg.eigvals(analysis.coupled_loop(M_ref, J_eff, WN))
modal = np.concatenate([np.roots([1, 3 * lm * WN, 3 * lm * WN**2, lm * WN**3]) for lm in lam])
gap = max(np.abs(poles - p).min() for p in modal)
print(f"its 15 poles against the five modes' roots: the largest gap is {gap:.1e} rad/s")
assert gap < 1e-9 * WN

rows, zetas = [], []
for lm in lam:
    r = np.roots([1, 3 * lm, 3 * lm, lm])
    pair = r[np.abs(r.imag) > 1e-9]
    zetas.append(-pair[0].real / abs(pair[0]) if len(pair) else 1.0)
    rows.append([f"{lm:.3f}", ",  ".join(f"{p.real:.2f}{p.imag:+.2f}j" for p in r),
                 f"{zetas[-1]:.2f}"])
nb.table(["λ", "poles, in units of ωn", "damping ratio ζ"], rows)
assert np.isclose(min(zetas), summary["coupled_zeta_min"], rtol=1e-9)

# %% [markdown]
# Not one of the five modes is the loop that was designed. The designed triple pole
# at $-\omega_n$ does not occur on the arm at all: the stiffest mode has a fast real
# pole and a damped pair, and the softest, which gets a fifth of its gain, is a
# lightly damped oscillation.
#
# How soft can a mode get? A cubic $s^3 + a s^2 + b s + c$ is stable when every
# coefficient is positive and $ab > c$ (Routh–Hurwitz). Here that reads:

# %%
lm_s = sp.symbols("lambda", positive=True)
nb.display(sp.solve((3 * lm_s) * (3 * lm_s) > lm_s, lm_s))
traj = nb.trajectory("ros_logo")
lam_path = []
for qq in traj["q"][::10]:
    Mi = np.linalg.inv(model.inertia(qq))
    lam_path.append(np.sort(np.linalg.eigvals(Mi / np.diag(Mi)).real))
lam_path = np.array(lam_path)
print(f"along the drawing, with the gains tuned where the arm is: softest mode "
      f"λ = {lam_path[:, 0].min():.3f} to {lam_path[:, 0].max():.3f}, stiffest "
      f"{lam_path[:, -1].min():.3f} to {lam_path[:, -1].max():.3f}")
r = np.roots([1, 3 * lam_path[:, 0].min(), 3 * lam_path[:, 0].min(), lam_path[:, 0].min()])
zeta_worst = min(-p.real / abs(p) for p in r)
print(f"at its softest, that mode's damping ratio is {zeta_worst:.2f}")
assert 1.3 < lam_path[:, 0].min() * 9 < 1.5 and zeta_worst < 0.1

# %% [markdown]
# Below $\lambda = 1/9$ a mode would oscillate for ever, however well each joint is
# tuned on its own. On this drawing the softest mode comes within about 1.4 times
# that, where its damping ratio is below 0.1: the arm's loop is stable everywhere it
# goes, but the margin comes from the masses, not from the tuning rule.
#
# The modes also explain what a step does to the whole arm. Stepping every joint by
# the same angle at once, as `analysis.py` does to publish its step response, and
# replaying that loop with the arm replaced by its linearisation $M(q_{\text{ref}})$:

# %%
def linear_step(step_rad, t_end=0.8, dt=1e-3):
    """analysis.simulate()'s loop, with the arm replaced by M at the tuning pose."""
    q, qd, ei = np.zeros(5), np.zeros(5), np.zeros(5)
    top = np.full(5, -np.inf)
    for k in range(int(t_end / dt)):
        e = (step_rad if k * dt >= 0.02 else 0.0) - q
        ei += e * dt
        qd = qd + M_inv @ (Kp_n * e + Ki_n * ei - Kd_n * qd) * dt
        q = q + qd * dt
        top = np.maximum(top, q)
    return (top - step_rad) / step_rad * 100


def arm_step(step_rad):
    """The full non-linear arm, through the repository's own simulation."""
    _, Qs, _, _ = analysis.simulate(model, Kp_n, Ki_n, Kd_n,
                                    lambda tt: q_ref + (step_rad if tt >= 0.02 else 0.0),
                                    t_end=0.8, q0=q_ref, gravity_ff=True)
    return ((Qs - q_ref).max(axis=0) - step_rad) / step_rad * 100


ov_linear = linear_step(np.deg2rad(5.0))
ov_small = arm_step(np.deg2rad(0.1))
ov_published = arm_step(np.deg2rad(5.0))
assert np.allclose(ov_published, summary["step_overshoot_ff_pct"], rtol=1e-9)
assert np.abs(ov_small - ov_linear).max() < 0.5
assert np.abs(ov_published - ov_linear)[:3].max() < 1          # the big joints agree at 5°
assert (ov_published[3:] < ov_linear[3:]).all() and 16 < (ov_linear - ov_published).max() < 18
assert (ov_published > pid_pct).all() and np.argmax(ov_published) == 4
nb.table(["joint", "one axis alone", "linear model of the arm", "the arm, 0.1° step",
          "the arm, 5° step (published)"],
         [[f"`{n}`", f"{pid_pct:.1f}%", f"{ov_linear[i]:.1f}%", f"{ov_small[i]:.1f}%",
           f"{ov_published[i]:.1f}%"] for i, n in enumerate(NAMES)])

# %% [markdown]
# The linear model predicts the non-linear arm's overshoot joint by joint, to a
# fraction of a percentage point once the step is small enough to keep the arm
# near its linearisation. At the published 5° the two wrist joints overshoot less
# than it predicts, by up to 17 points; the gap closes as the step shrinks, so it is
# the step's size, not the model, that separates them. Either way the lesson is
# the same: every joint overshoots more than one axis alone would, most of all the
# wrist, whose motion the soft modes are made of.

# %%
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.3), gridspec_kw={"width_ratios": [1, 1.1]})
sweep = np.geomspace(0.03, 6, 900)
loci = [np.roots([1, 3 * sweep[0], 3 * sweep[0], sweep[0]])]
for lm in sweep[1:]:
    # Each branch continues to the nearest root: sorting them instead makes the
    # branches swap places where they cross, and draws lines that are not there.
    roots = np.roots([1, 3 * lm, 3 * lm, lm])
    loci.append(np.array(min(permutations(roots),
                             key=lambda order: np.abs(np.array(order) - loci[-1]).sum())))
loci = np.array(loci)
for j in range(3):
    a1.plot(loci[:, j].real, loci[:, j].imag, color=MUTED, lw=1.0)
a1.axvline(0, color=INK2, lw=0.8)
a1.plot([-1], [0], "s", color=INK2, ms=7)
a1.annotate("λ = 1: the designed\ntriple pole", (-1, 0), xytext=(-2.3, 0.42),
            fontsize=8, color=INK2, ha="center",
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
a1.plot([0, 0], [1 / np.sqrt(3), -1 / np.sqrt(3)], "x", color=INK2, ms=7)
a1.annotate("λ = 1/9: unstable", (0, 1 / np.sqrt(3)), xytext=(6, 4),
            textcoords="offset points", fontsize=8, color=INK2)
for i, lm in enumerate(lam):
    r = np.roots([1, 3 * lm, 3 * lm, lm])
    a1.plot(r.real, r.imag, "o", color=C[i], ms=6, label=f"λ = {lm:.2f}, ζ = {zetas[i]:.2f}")
a1.set_xlim(-3.2, 0.45)
a1.set_ylim(-1.15, 1.15)
a1.set_xlabel("real part / ωn")
a1.set_ylabel("imaginary part / ωn")
a1.set_title("Where the arm's modes put the poles", loc="left")
a1.legend(fontsize=8, loc="lower left")
a1.grid(True, alpha=0.9)
x = np.arange(5)
a2.bar(x - 0.2, ov_linear, 0.38, color=C[0], label="linear model of the arm")
a2.bar(x + 0.2, ov_published, 0.38, color=C[1], label="the non-linear arm, 5° step")
a2.axhline(pid_pct, color=INK2, lw=1.0, ls="--", label=f"one axis alone, {pid_pct:.1f}%")
a2.set_xticks(x, [f"j{i + 1}" for i in range(5)])
a2.set_ylabel("step overshoot (%)")
a2.set_title("Every joint stepped at once, gravity fed forward", loc="left")
a2.legend(fontsize=8, loc="upper left")
a2.grid(True, axis="y", alpha=0.9)
nb.show(fig, export="27_coupled_modes.png")

# %% [markdown]
# ## 5 · The sampled loop, and the rate ceiling
#
# The controller does not act continuously. It runs at a fixed rate $1/T$: it reads
# the joint, computes a torque, and holds that torque until the next reading.
# Between readings a joint under constant torque integrates it exactly — the
# **zero-order hold**:
#
# $$q_{k+1} = q_k + T\dot q_k + \frac{T^2}{2J}\tau_k, \qquad
#   \dot q_{k+1} = \dot q_k + \frac{T}{J}\tau_k .$$
#
# Seen only at the samples, the double integrator becomes two pulse transfer
# functions, from torque to position and to velocity:

# %%
z = sp.symbols("z")
T_s = sp.symbols("T", positive=True)
Phi = sp.Matrix([[1, T_s], [0, 1]])
Gam = sp.Matrix([T_s**2 / 2, T_s])
resolvent = (z * sp.eye(2) - Phi).inv()
nb.display(sp.factor((sp.Matrix([[1, 0]]) * resolvent * Gam)[0]),
           sp.factor((sp.Matrix([[0, 1]]) * resolvent * Gam)[0]))

# %% [markdown]
# (Both per unit inertia.) The position has a zero at $z = -1$. A torque that flips
# sign every sample — the fastest thing a sampled controller can do — leaves the
# position where it was at every reading; only the velocity sees it. That zero
# decides where the loop breaks.
#
# The loop itself, with the integral updated by the new error before the torque
# uses it, as `analysis.simulate()` does. Time in samples, and the gains in the
# units that makes them: $K_pT^2/J = 3x^2$, $K_dT/J = 3x$, $K_iT^3/J = x^3$ with
# $x = \omega_n T$. For a mode that receives $\lambda$ of its gain, the state
# $(q,\; T\dot q,\; \int e/T)$ moves from one sample to the next by:

# %%
x_s = sp.symbols("x", positive=True)
A3 = sp.Matrix([[1, 1, 0], [0, 1, 0], [-1, 0, 1]])
B3 = sp.Matrix([sp.Rational(1, 2), 1, 0])
k3 = sp.Matrix([[-(3 * x_s**2 + x_s**3), -3 * x_s, x_s**3]])
char_z = sp.expand((z * sp.eye(3) - (A3 + lm_s * B3 * k3)).det())
nb.display(sp.collect(char_z, z))
at_minus_one, at_plus_one = sp.factor(char_z.subs(z, -1)), sp.factor(char_z.subs(z, 1))
nb.display(sp.Eq(sp.Symbol("p(-1)"), at_minus_one), sp.Eq(sp.Symbol("p(1)"), at_plus_one))
edge = sp.solve(at_minus_one, x_s)[0]
nb.display(sp.Eq(sp.Symbol("x_max"), edge))
assert sp.simplify(at_minus_one - 4 * (3 * lm_s * x_s - 2)) == 0
assert sp.simplify(edge - sp.Rational(2, 3) / lm_s) == 0

# %% [markdown]
# $p(1) = \lambda x^3$ never vanishes, so no pole ever leaves through $z = 1$. At
# $z = -1$ the position terms cancel — the zero of the hold — and what is left
# belongs to the derivative alone: $p(-1) = 4(3\lambda x - 2)$. A pole leaves the
# unit circle through $-1$ when $\lambda\,\omega_n T = 2/3$, which is where the
# derivative term by itself, $\dot q_{k+1} = (1 - \lambda K_dT/J)\,\dot q_k$, starts
# reversing the velocity it measured and making it larger. The loop flips sign every
# sample from there on, and grows.
#
# Two roads lead to that pole: a slower rate (larger $T$), or a higher bandwidth.
# The numbers, checked first by brute force on the spectral radius:

# %%
def mode_map(x_, lm=1.0, c=0.5):
    """One mode's sampled loop, sample to sample. c = 1/2 is the exact hold."""
    A = np.array([[1, 1, 0], [0, 1, 0], [-1, 0, 1]], float)
    k = np.array([-(3 * x_**2 + x_**3), -3 * x_, x_**3])
    return A + lm * np.outer([c, 1, 0], k)


def rho(x_, lm=1.0, c=0.5):
    """Its spectral radius: the factor the slowest-dying part changes by per sample."""
    return np.abs(np.linalg.eigvals(mode_map(x_, lm, c))).max()


def bisect(f, lo=1e-3, hi=2.0):
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) < 1 else (lo, mid)
    return lo


one_axis = bisect(rho)
print(f"one joint on its own: stable while ωnT < {one_axis:.6f}")
assert abs(one_axis - 2 / 3) < 1e-9
arm_edge = 2 / (3 * lam[0])
print(f"the arm at the tuning pose: its stiffest mode gets λ = {lam[0]:.3f} of its gain, "
      f"so ωnT < 2/(3λ) = {arm_edge:.4f}")
assert abs(arm_edge - summary["sampled_boundary_wnT"]) < 1e-9
assert lam[0] > 2 and round(arm_edge, 3) == 0.316
lm_all, vecs = np.linalg.eig(M_inv @ np.diag(J_eff))
shape = vecs[:, np.argmax(lm_all.real)].real
shape = shape / shape[np.argmax(np.abs(shape))]
print("the stiffest mode moves the joints in the proportions",
      ", ".join(f"{n} {v:+.2f}" for n, v in zip(NAMES, shape)))
assert np.argmax(np.abs(shape)) == 4 and (shape[2:4] < 0).all()
band = 2 / (3 * lam_path[:, -1])
print(f"along the drawing: the boundary moves between {band.min():.3f} and {band.max():.3f}")
assert np.allclose([band.min(), band.max()], summary["sampled_boundary_band_wnT"], rtol=1e-6)

# %% [markdown]
# The same coupling that detunes the modes halves the ceiling: one joint alone
# would be stable up to $\omega_n T = 2/3$, and the arm's stiffest mode, which gets
# more than twice its designed gain, goes unstable at 0.316. That mode is the
# wrist: `joint_5`, with `joint_3` and `joint_4` moving against it. The joints with
# the least inertia of their own are the ones the others push hardest, and they set
# the ceiling for the whole arm. `analysis.py` finds the
# arm's boundary by bisecting on all fifteen poles at once, with no modes assumed,
# and the closed form agrees with it to within $10^{-9}$.
#
# The rate study swept three rates against three bandwidths on the non-linear arm.
# Its verdicts, against the rule that a loop fails if $\omega_n T$ passes the
# boundary anywhere on the drawing:

# %%
rows, agree = [], []
for r in sorted(rate_study["runs"].values(), key=lambda r: (r["rate"], r["wn"])):
    x_ = r["wn"] / r["rate"]
    past = x_ > band.min()
    agree.append(past == (r["verdict"] != "ok"))
    rows.append([f"{r['rate']:.0f} Hz", f"{r['wn']:.0f}", f"{x_:.2f}",
                 "past it" if past else "inside", r["verdict"]])
nb.table(["rate", "ωn (rad/s)", "ωnT", "against the boundary", "rate study"], rows)
assert all(agree)
print("the rule sorts all nine runs the way the non-linear simulation did")

# %%
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1, 1.25]})
circle = np.exp(1j * np.linspace(0, 2 * np.pi, 400))
a1.plot(circle.real, circle.imag, color=INK2, lw=1.0)
xs_arm = np.linspace(0.004, 0.5, 300)
P = np.array([np.linalg.eigvals(analysis.coupled_loop(M_ref, J_eff, v, dt=1.0))
              for v in xs_arm])
# The ramp without its two lightest steps, which vanish against the background.
ramp_x = LinearSegmentedColormap.from_list("ramp_x", BLUES[2:])
dots = a1.scatter(P.real.ravel(), P.imag.ravel(), c=np.repeat(xs_arm, P.shape[1]),
                  cmap=ramp_x, vmin=0, vmax=0.5, s=5, lw=0)
fig.colorbar(dots, ax=a1, shrink=0.75, label="ωn T")
a1.plot([-1], [0], "o", color=C[1], ms=7, mfc="white", mew=1.8)
a1.annotate(f"the wrist mode leaves\nhere, at ωnT = {arm_edge:.3f}", (-1, 0),
            xytext=(-1.75, 0.55), fontsize=8, color=INK2,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
a1.set_aspect("equal")
a1.set_xlim(-2.0, 1.15)
a1.set_ylim(-1.15, 1.15)
a1.set_xlabel("real")
a1.set_ylabel("imaginary")
a1.set_title("The arm's 15 poles as ωnT grows to 0.5", loc="left")
a1.grid(True, alpha=0.9)
xs = np.linspace(0.004, 0.8, 500)

a2.axvspan(band.min(), band.max(), color=C[1], alpha=0.12, lw=0)
a2.annotate(f"the boundary along\nthe drawing, {band.min():.3f}\nto {band.max():.3f}",
            (band.max(), 1.12), xytext=(8, 0), textcoords="offset points", fontsize=8,
            color=INK2)
a2.plot(xs, [rho(v) for v in xs], color=C[0], label="one joint alone")
a2.plot(xs, [max(rho(v, lm) for lm in lam) for v in xs], color=C[1],
        label="the arm, at the tuning pose")
a2.axhline(1, color=INK2, lw=0.8)
runs = {}
for r in rate_study["runs"].values():
    runs.setdefault(round(r["wn"] / r["rate"], 4), []).append(r)
for x_, rs in sorted(runs.items()):
    failed = any(r["verdict"] != "ok" for r in rs)
    y = min(max(rho(x_, lm) for lm in lam), 1.75)
    a2.plot(x_, y, "X" if failed else "o", color=C[1] if failed else C[0], ms=8,
            mfc="white" if failed else None, mew=1.8)
    if failed:
        name = ", ".join(f"{r['rate']:.0f} Hz and {r['wn']:.0f} rad/s" for r in rs)
        a2.annotate(name + (", off scale" if y >= 1.75 else ""), (x_, y), xytext=(-8, 0),
                    textcoords="offset points", ha="right", va="center", fontsize=7,
                    color=INK2)
a2.plot([], [], "o", color=C[0], ms=7, label="rate study: worked")
a2.plot([], [], "X", color=C[1], ms=8, mfc="white", mew=1.8, label="rate study: failed")
a2.set_ylim(0.6, 1.8)
a2.set_xlim(0, 0.85)
a2.set_xlabel("ωn T  (bandwidth, rad/s, × sample period, s)")
a2.set_ylabel("growth per sample, largest pole |z|")
a2.set_title("Stable below 1: the rate study's nine runs", loc="left")
a2.legend(fontsize=8, loc="lower left")
a2.grid(True, alpha=0.9)
nb.show(fig, export="28_rate_ceiling.png")

# %% [markdown]
# As $\omega_n T$ grows the poles spread out from $z = 1$, and each mode sends one
# of its three along the real axis towards $-1$; the wrist mode's gets there first.
# On the right, the six runs that worked (dots; pairs of them share a value of
# $\omega_n T$) all sit left of the band, and the three that failed (crosses) sit
# inside it or beyond. The 500 Hz, 160 rad/s run is the close one: at
# $\omega_n T$ = 0.32 it is past the boundary at the tuning pose, but inside it over
# part of the drawing.
#
# The same poles, moving, for the whole arm:

# %%
fig, ax = plt.subplots(figsize=(5.6, 5.2))
ax.plot(circle.real, circle.imag, color=INK2, lw=1.0)
dots = ax.scatter([], [], s=28)
label = ax.text(-1.75, 1.2, "", fontsize=9, color=INK2)
ax.set_aspect("equal")
ax.set_xlim(-1.9, 1.2)
ax.set_ylim(-1.3, 1.35)
ax.grid(True, alpha=0.9)
ax.set_title("The arm's 15 poles as the rate falls", loc="left")
frames_x = np.linspace(0.02, 0.5, 97)


def frame_poles(k):
    v = frames_x[k]
    P = np.linalg.eigvals(analysis.coupled_loop(M_ref, J_eff, v, dt=1.0))
    out = np.abs(P) > 1
    dots.set_offsets(np.c_[P.real, P.imag])
    dots.set_color([C[1] if o else C[0] for o in out])
    label.set_text(f"ωnT = {v:.3f}: {out.sum()} of 15 outside the circle")
    return dots, label


nb.animate(animation.FuncAnimation(fig, frame_poles, frames=len(frames_x), interval=60),
           fps=15)

# %% [markdown]
# ## 6 · The non-linear arm past the ceiling
#
# Everything in section 5 is linear, and at one pose. The rate study ran the full
# non-linear arm, integrated at 4 kHz with the torque limited to 20 N·m. Here is
# that loop, holding the tuning pose, started a billionth of a radian away from it.
# If the analysis is right, the error has to grow — or shrink — by the largest pole's
# magnitude every sample, until something non-linear stops it.
#
# One detail has to match first. The simulation does not integrate between samples
# exactly: it takes $N$ = 4000/rate small steps, each updating the velocity and then
# the position, which turns the hold's $T^2/2$ into $T^2(N+1)/(2N)$. The boundary
# moves with it:

# %%
c_s = sp.symbols("c", positive=True)
char_c = sp.expand((z * sp.eye(3) - (A3 + lm_s * sp.Matrix([c_s, 1, 0]) * k3)).det())
nb.display(sp.Eq(sp.Symbol("p(-1)"), sp.factor(char_c.subs(z, -1))))
for rate in (200, 500, 1000):
    N = 4000 // rate
    moved = bisect(lambda v: max(rho(v, lm, (N + 1) / (2 * N)) for lm in lam))
    print(f"  {rate:4d} Hz, N = {N:2d}: boundary {moved:.4f} against {arm_edge:.4f} "
          f"for the exact hold")


# %%
def hold_pose(rate, wn, t_end, tau_max=20.0):
    """The rate study's loop, holding the tuning pose, from a tiny offset."""
    Kp_, Ki_, Kd_ = analysis.tune(model, q_ref, wn=wn)
    dt, every = 1 / 4000, 4000 // rate
    q = q_ref + np.random.default_rng(0).normal(scale=1e-9, size=5)
    qd, ei, tau = np.zeros(5), np.zeros(5), np.zeros(5)
    err, limited, tip = [], [], []
    for k in range(int(round(t_end / dt))):
        if k % every == 0:
            e = q_ref - q
            ei += e * every * dt
            tau = np.clip(Kp_ * e + Ki_ * ei - Kd_ * qd + model.gravity(q), -tau_max, tau_max)
            err.append(np.linalg.norm(e))
            limited.append(np.abs(tau).max() >= tau_max)
            tip.append(np.linalg.norm(chain.tcp(q) - tip_ref))
        qd = qd + model.forward(q, qd, tau) * dt
        q = q + qd * dt
    return np.array(err), np.array(limited), np.array(tip)


tip_ref = chain.tcp(q_ref)
LINE = 0.3e-3                     # the width of the line a tattoo needle lays, m


cases = [(200, 80, 0.3), (500, 160, 0.7), (1000, 160, 0.3)]
growth = {}
for rate, wn, t_end in cases:
    err, limited, tip = hold_pose(rate, wn, t_end)
    N = 4000 // rate
    predicted = max(rho(wn / rate, lm, (N + 1) / (2 * N)) for lm in lam)
    k = np.arange(len(err))
    # Fit the second half of the stretch before anything non-linear can act: the
    # first half is where the faster modes are still dying out.
    linear = (err < 1e-4) & ~np.cumsum(limited).astype(bool)
    stop = len(err) if linear.all() else int(np.argmin(linear))
    fit = slice(stop // 2, stop)
    measured = np.exp(np.polyfit(k[fit], np.log(err[fit]), 1)[0])
    growth[(rate, wn)] = (err, limited, predicted, measured, stop // 2)
    print(f"{rate:4d} Hz, {wn:3d} rad/s: error × {measured:.4f} per sample, "
          f"predicted × {predicted:.4f}"
          + (f"; torque limit reached after {np.argmax(limited)} samples"
             if limited.any() else "; never near the torque limit"))
    assert abs(measured - predicted) < 2e-3
    assert limited.any() == (predicted > 1)          # every unstable run ends at the limit
    if limited.any():                                # and never comes back inside the line
        after = tip[np.argmax(limited):]
        print(f"      after it, the needle tip is {after.min() * 1e3:.1f} to "
              f"{after.max() * 1e3:.1f} mm from where it should be")
        assert after.min() > LINE
    if predicted < 1:                      # the slowest pole to die out is the softest mode's
        assert np.argmax([rho(wn / rate, lm, (N + 1) / (2 * N)) for lm in lam]) == 4

# %% [markdown]
# The non-linear arm grows or decays at the rate the linear loop predicts, to the
# third decimal. Where the loop is unstable the error grows geometrically until a
# command reaches the torque limit, and from there the limit, not the loop, decides
# what happens: the needle tip never comes back inside a 0.3 mm line. That is the
# state the rate study calls *saturated*. Where the loop is stable, the slowest pole
# to die out is the softest mode's.

# %%
fig, ax = plt.subplots(figsize=(9, 4.2))
for (rate, wn), colour in zip(growth, (C[1], C[3], C[0])):
    err, limited, predicted, measured, k0 = growth[(rate, wn)]
    tk = np.arange(len(err)) / rate
    ax.semilogy(tk, err, color=colour, label=f"{rate} Hz, ωn = {wn}: ωnT = {wn / rate:.2f}")
    kk = np.arange(len(err))
    ax.semilogy(tk, err[k0] * predicted ** (kk - k0), color=colour, lw=0.9, ls="--")
    if limited.any():
        i = np.argmax(limited)
        ax.plot(tk[i], err[i], "o", color=colour, ms=7, mfc="white", mew=1.8)
        ax.annotate("torque limit", (tk[i], err[i]), xytext=(6, -12),
                    textcoords="offset points", fontsize=8, color=INK2)
ax.set_ylim(1e-13, 10)
ax.set_xlabel("time (s)")
ax.set_ylabel("joint error |e| (rad)")
ax.set_title("From a billionth of a radian: measured (solid) against predicted (dashed)",
             loc="left")
ax.legend(fontsize=8, loc="lower right")
ax.grid(True, alpha=0.9)
nb.show(fig, export="29_past_the_ceiling.png")

# %% [markdown]
# What this means for hardware. The ceiling is set by the product of bandwidth and
# sample period, and by how unevenly the arm's mass matrix spreads the gains. At the
# bandwidth the studies recommend:

# %%
wn_rec = 160.0
slowest = wn_rec / band.min()
print(f"at ωn = {wn_rec:.0f} rad/s the loop needs a rate above {slowest:.0f} Hz on this "
      f"drawing; 1 kHz leaves a factor of {1000 / slowest:.1f} in hand")
assert 500 < slowest < 560 and 1.8 < 1000 / slowest < 2.0

# %% [markdown]
# So a 500 Hz servo interface is not enough at that bandwidth — the rate study's
# failure at 500 Hz, 160 rad/s is this line — and 1 kHz clears it with a factor of
# about two. A bus that cannot sustain it puts the ceiling back where the rate
# study found it.
