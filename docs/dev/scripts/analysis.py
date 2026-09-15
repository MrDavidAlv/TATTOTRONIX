"""Everything DEVELOPMENT.md quotes, computed once and cached.

Run this, then `figures.py`. Every number in the document comes out of the npz
this writes, so the prose cannot drift away from the model.
"""

import json
from pathlib import Path

import numpy as np

import rospath as rp

_print = print
def print(*a, **k):
    k.setdefault('flush', True)
    _print(*a, **k)
from dynamics import load as load_dyn

OUT = Path(__file__).resolve().parents[1] / "data"
OUT.mkdir(exist_ok=True)

# The panel, in the arm's own frame. Gazebo puts the arm on a 750 mm bench and
# the panel 2.5 mm above it, so in the arm frame the work surface is at z = +5 mm.
PANEL_Z = 0.005
PANEL_X = 0.21
PANEL_W, PANEL_H = 0.20, 0.14

CONTROL_HZ = 200.0          # matches the controller manager rate
FEED_MARK = 0.006           # m/s with the needle in the work
FEED_TRAVEL = 0.060         # m/s clear of the work


def workspace(chain, n=60_000, seed=0):
    rng = np.random.default_rng(seed)
    Q = rng.uniform(chain.lower, chain.upper, size=(n, chain.n))
    P = np.array([chain.tcp(q) for q in Q])
    return Q, P


def panel_reachability(chain, nx=33, ny=23):
    """Can the pen stand normal to the panel at each point, and how well."""
    xs = PANEL_X + np.linspace(-PANEL_W / 2, PANEL_W / 2, nx)
    ys = np.linspace(-PANEL_H / 2, PANEL_H / 2, ny)
    ok = np.zeros((ny, nx), bool)
    mu = np.full((ny, nx), np.nan)
    margin = np.full((ny, nx), np.nan)
    seed = np.array([0.0, 0.6, -0.9, 0.0, -1.2])
    for j, y in enumerate(ys):
        q_prev = seed.copy()
        for i, x in enumerate(xs):
            q, conv, _ = chain.ik(np.array([x, y, PANEL_Z]), [0, 0, -1], q_prev)
            if conv:
                ok[j, i] = True
                mu[j, i] = chain.manipulability(q)
                margin[j, i] = np.min(np.minimum(q - chain.lower, chain.upper - q))
                q_prev = q
            else:
                q_prev = seed.copy()
    return xs, ys, ok, mu, margin


def path_to_world(P_mm):
    """Artwork millimetres -> arm frame metres on the panel."""
    P = np.empty_like(P_mm, dtype=float)
    P[:, 0] = PANEL_X + P_mm[:, 0] / 1000.0
    P[:, 1] = P_mm[:, 1] / 1000.0
    P[:, 2] = PANEL_Z + P_mm[:, 2] / 1000.0
    return P


def time_parameterise(P, kind):
    """Constant feed per move type; returns cumulative time at each point."""
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    cutting = (kind[:-1] == 1) & (kind[1:] == 1)
    v = np.where(cutting, FEED_MARK, FEED_TRAVEL)
    dt = d / v
    return np.concatenate([[0.0], np.cumsum(dt)])


def solve_path(chain, P):
    """IK along the whole path, warm started from the previous point."""
    Q = np.zeros((len(P), chain.n))
    res = np.zeros(len(P))
    conv = np.zeros(len(P), bool)
    q = np.array([0.0, 0.6, -0.9, 0.0, -1.2])
    for i, p in enumerate(P):
        q, c, r = chain.ik(p, [0, 0, -1], q)
        Q[i], conv[i], res[i] = q, c, r
    return Q, conv, res


# How wide the mark is drawn on the panel.
#
# The smallest singular value of the task Jacobian along the path turns out to
# depend only on how far the arm reaches in +x, which is dx + width/2: 120 mm
# centred and 150 mm offset by 15 give the same 0.0484 because both reach
# 75 mm. It falls as the drawing moves out, so the mark stays centred, and 150
# on a 200 mm panel is the largest size that keeps the condition number in the
# thirties.
LOGO_WIDTH_MM = 150.0

TUNE_WN = 20.0      # loop bandwidth the cached run is tuned at, rad/s


def tune(model, q_ref, wn=TUNE_WN, zeta=1.0):
    """Gains by pole placement on the effective joint inertia.

    Each axis is a double integrator, J qdd = tau, so a PID closes it as

        J s^3 + Kd s^2 + Kp s + Ki

    Placing all three poles at -wn gives (s + wn)^3 and therefore

        Kd = 3 J wn,   Kp = 3 J wn^2,   Ki = J wn^3

    which has no overshoot by construction. An earlier version used the
    second-order PD rule with an integral term bolted on as a fraction of Kp;
    that put the integral time at 0.33 s against a loop time constant of
    0.05 s, and the step response overshot by 30 to 75 percent.

    J is the *effective* inertia, 1 / (M^-1)_ii, not the diagonal of M. For
    joints 2 and 3 the two differ by a factor of two: the diagonal says how
    much inertia the axis carries if every other axis is frozen, and nothing
    freezes the others. Using the diagonal detunes those axes by that factor.

    The point of all of this is that every gain traces back to a measured
    inertia and to one bandwidth decision, rather than to a knob someone
    turned until it stopped oscillating.
    """
    J = 1.0 / np.diag(np.linalg.inv(model.inertia(q_ref)))
    Kp = 3.0 * J * wn ** 2
    Kd = 3.0 * J * wn
    Ki = J * wn ** 3
    return Kp, Ki, Kd


def simulate(model, Kp, Ki, Kd, q_ref_fn, t_end, dt=1.0 / 1000, q0=None,
             gravity_ff=True, tau_max=20.0):
    """Closed loop with the full nonlinear dynamics in the plant."""
    n = model.n
    q = np.array(q0 if q0 is not None else q_ref_fn(0.0), float)
    qd = np.zeros(n)
    ei = np.zeros(n)
    steps = int(t_end / dt)
    ctrl_every = max(int(round((1.0 / CONTROL_HZ) / dt)), 1)
    tau = np.zeros(n)
    T, Q, QR, TAU = [], [], [], []
    for k in range(steps):
        t = k * dt
        qr = q_ref_fn(t)
        if k % ctrl_every == 0:
            e = qr - q
            ei += e * (ctrl_every * dt)
            tau = Kp * e + Ki * ei - Kd * qd
            if gravity_ff:
                tau = tau + model.gravity(q)
            tau = np.clip(tau, -tau_max, tau_max)
        qdd = model.forward(q, qd, tau)
        qd = qd + qdd * dt
        q = q + qd * dt
        T.append(t); Q.append(q.copy()); QR.append(qr.copy()); TAU.append(tau.copy())
    return (np.array(T), np.array(Q), np.array(QR), np.array(TAU))


def main():
    chain, model = load_dyn()
    out = {}

    print("workspace...")
    Q, P = workspace(chain)
    out["ws_P"] = P

    print("panel reachability...")
    xs, ys, ok, mu, margin = panel_reachability(chain)
    out |= {"panel_xs": xs, "panel_ys": ys, "panel_ok": ok,
            "panel_mu": mu, "panel_margin": margin}

    print("toolpath...")
    mask, grid = rp.ros_logo_mask(LOGO_WIDTH_MM)
    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    out |= {"path_mm": P_mm, "path_world": Pw, "path_kind": kind, "path_t": t,
            "mask": mask, "mask_xs": grid[0], "mask_ys": grid[1]}

    print(f"  {len(Pw)} points, {t[-1]:.0f} s at {FEED_MARK*1000:.0f} mm/s marking")

    print("ik along the path...")
    Qp, conv, res = solve_path(chain, Pw)
    out |= {"path_q": Qp, "path_conv": conv, "path_res": res}
    print(f"  converged at {conv.mean()*100:.1f}% of points, worst residual {res.max():.2e}")

    mu_path = np.array([chain.manipulability(q) for q in Qp])
    out["path_mu"] = mu_path

    print("gravity and inertia along the path...")
    out["path_grav"] = np.array([model.gravity(q) for q in Qp[::10]])
    out["path_Mdiag"] = np.array([np.diag(model.inertia(q)) for q in Qp[::10]])

    print("pid tuning...")
    q_ref = Qp[len(Qp) // 2]
    Kp, Ki, Kd = tune(model, q_ref)
    out |= {"Kp": Kp, "Ki": Ki, "Kd": Kd, "q_ref_tune": q_ref}
    print("  Kp", np.round(Kp, 3))
    print("  Ki", np.round(Ki, 3))
    print("  Kd", np.round(Kd, 4))

    print("step response...")
    step = np.deg2rad(5.0)
    q_start = q_ref.copy()
    for gff, tag in [(True, "ff"), (False, "noff")]:
        T, Qs, QR, TAU = simulate(
            model, Kp, Ki, Kd,
            lambda tt: q_start + (step if tt >= 0.02 else 0.0),
            t_end=0.8, q0=q_start, gravity_ff=gff)
        out |= {f"step_T_{tag}": T, f"step_Q_{tag}": Qs,
                f"step_QR_{tag}": QR, f"step_TAU_{tag}": TAU}

    print("tracking the logo...")
    # follow one dot: long enough to show steady-state tracking and the
    # plunge/retract transients, short enough to integrate at 1 kHz.
    stop = int(np.searchsorted(t, t[0] + 12.0))
    win = slice(0, max(stop, 50))
    tt, QQ = t[win] - t[win][0], Qp[win]
    def ref(s):
        s = min(max(s, 0.0), tt[-1])
        return np.array([np.interp(s, tt, QQ[:, i]) for i in range(chain.n)])
    T, Qs, QR, TAU = simulate(model, Kp, Ki, Kd, ref, t_end=float(tt[-1]), q0=QQ[0])
    out |= {"trk_T": T, "trk_Q": Qs, "trk_QR": QR, "trk_TAU": TAU}

    # Cartesian tracking error, split by what the needle is doing.
    #
    # A single mean over the whole window is not a number about the drawing.
    # Travel runs at ten times the marking feed, so its velocity lag is ten
    # times larger, and the mean then reports how much travel happened to fall
    # inside the window rather than how well the arm draws. Only the marking
    # error ever touches skin.
    Ts = T[::20]
    err = np.array([np.linalg.norm(chain.tcp(a) - chain.tcp(b))
                    for a, b in zip(Qs[::20], QR[::20])])
    marking = (kind[win][:-1] == 1) & (kind[win][1:] == 1)
    down = np.interp(Ts, tt[:-1], marking.astype(float)) > 0.5

    # The needle enters the work straight off a plunge, which is a travel move
    # at ten times the feed, so the loop is still settling when marking starts.
    # `settled` drops one second-order settling time, 4/wn, after each entry, so
    # the steady figure is about drawing and the transient is reported on its
    # own rather than averaged into it.
    entry = np.flatnonzero(down & ~np.concatenate([[False], down[:-1]]))
    settled = down.copy()
    for i in entry:
        settled &= ~((Ts >= Ts[i]) & (Ts < Ts[i] + 4.0 / TUNE_WN))
    out |= {"trk_cart_err": err, "trk_cart_T": Ts, "trk_cart_down": down,
            "trk_cart_settled": settled}
    print(f"  cartesian error, marking settled  mean {err[settled].mean()*1e6:7.1f} um  "
          f"max {err[settled].max()*1e6:8.1f} um")
    print(f"                   marking all      mean {err[down].mean()*1e6:7.1f} um  "
          f"max {err[down].max()*1e6:8.1f} um   ({len(entry)} needle entries)")
    print(f"                   travel           mean {err[~down].mean()*1e6:7.1f} um  "
          f"max {err[~down].max()*1e6:8.1f} um")

    np.savez_compressed(OUT / "analysis.npz", **out)

    summary = {
        "tcp_zero_mm": (chain.tcp(np.zeros(chain.n)) * 1000).round(2).tolist(),
        "path_points": int(len(Pw)),
        "path_time_s": float(t[-1]),
        "path_marked_mm": rp.lengths(P_mm, kind)[0],
        "path_travel_mm": rp.lengths(P_mm, kind)[1],
        "ik_converged_pct": float(conv.mean() * 100),
        "ik_worst_residual": float(res.max()),
        "panel_reachable_pct": float(ok.mean() * 100),
        "mu_path_min": float(np.nanmin(mu_path)),
        "mu_path_max": float(np.nanmax(mu_path)),
        "Kp": Kp.round(4).tolist(), "Ki": Ki.round(4).tolist(), "Kd": Kd.round(5).tolist(),
        "cart_err_marking_settled_mean_um": float(err[settled].mean() * 1e6),
        "cart_err_marking_settled_max_um": float(err[settled].max() * 1e6),
        "cart_err_marking_mean_um": float(err[down].mean() * 1e6),
        "cart_err_marking_max_um": float(err[down].max() * 1e6),
        "cart_err_travel_mean_um": float(err[~down].mean() * 1e6),
        "cart_err_travel_max_um": float(err[~down].max() * 1e6),
        "tau_peak_Nm": float(np.abs(TAU).max()),
        "grav_peak_Nm": float(np.abs(out["path_grav"]).max()),
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))
    print("\nwrote", OUT / "analysis.npz", "and summary.json")
    return summary


if __name__ == "__main__":
    s = main()
    print()
    for k, v in s.items():
        print(f"  {k:24} {v}")
