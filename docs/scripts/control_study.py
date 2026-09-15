"""Why plain PID cannot tattoo, and what fixes it.

The first tuning left 353 um of mean cartesian error while marking. That is not
noise and it is not a bad gain choice: it is the velocity lag every
proportional loop has when asked to follow a ramp. For a critically damped
second order loop of bandwidth wn tracking a constant feed v, the steady state
lag is v / wn, which at 6 mm/s and 20 rad/s is 300 um. A tattoo line is
0.3 mm wide; an error of that size is a different drawing.

Two ways out, and they are not equivalent:

  * raise wn - bounded by the 200 Hz controller rate and by torque;
  * feed the reference forward so the loop is not asked to produce the
    velocity from error in the first place.

This sweeps both and reports what each buys. Feedforward wins by two orders of
magnitude and costs nothing but arithmetic, which is the expected answer and
the reason trajectory controllers ship with it.
"""

import json
from pathlib import Path

import numpy as np

import rospath as rp
from analysis import (CONTROL_HZ, LOGO_WIDTH_MM, PANEL_Z, needle_state,
                      path_to_world, solve_path, time_parameterise, tune)
from dynamics import load as load_dyn

OUT = Path(__file__).resolve().parents[1] / "data"

MODES = {
    "pid": dict(grav=False, vff=False, aff=False),
    "pid+g": dict(grav=True, vff=False, aff=False),
    "pid+g+v": dict(grav=True, vff=True, aff=False),
    "computed torque": dict(grav=True, vff=True, aff=True),
}


WINDOW_S = 12.0     # length of the studied window, s


def window_start(P_mm, kind, t, window_s=WINDOW_S):
    """Index where the busiest `window_s` of the path begins.

    Busiest means the most needle entries. Found from the path rather than
    fixed, so it follows the artwork instead of a hardcoded time.

    The path starts with the nine dots, which are convex blobs filled in one
    pass each, and a study run there would be measuring the easy part of the
    drawing. The stress is inside the letters: every scanline that meets the
    counter of the R or the O has to lift and re-enter, and it is those entries
    that the loop tracks worst.
    """
    entry_t = t[np.flatnonzero((kind[:-1] != rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK))]
    if len(entry_t) == 0:
        return 0
    counts = [(entry_t < s0 + window_s).sum() - (entry_t < s0).sum() for s0 in entry_t]
    return int(np.searchsorted(t, entry_t[int(np.argmax(counts))]))


def run(model, chain, tt, QQ, QD, QDD, wn, mode, dt=1.0 / 1000, tau_max=20.0):
    """Returns (errors, peak torques, sample times); errors sampled every 20 steps."""
    Kp, Ki, Kd = tune(model, QQ[len(QQ) // 2], wn=wn)
    cfg = MODES[mode]
    n = model.n
    q = QQ[0].copy()
    qd = QD[0].copy()
    ei = np.zeros(n)
    ctrl_every = max(int(round((1.0 / CONTROL_HZ) / dt)), 1)
    tau = np.zeros(n)
    steps = int(tt[-1] / dt)
    errs, taus, ts = [], [], []
    for k in range(steps):
        t = k * dt
        qr = np.array([np.interp(t, tt, QQ[:, i]) for i in range(n)])
        qdr = np.array([np.interp(t, tt, QD[:, i]) for i in range(n)])
        qddr = np.array([np.interp(t, tt, QDD[:, i]) for i in range(n)])
        if k % ctrl_every == 0:
            e = qr - q
            ed = (qdr - qd) if cfg["vff"] else -qd
            ei += e * (ctrl_every * dt)
            tau = Kp * e + Ki * ei + Kd * ed
            if cfg["aff"]:
                tau = tau + model.inertia(q) @ qddr + model.coriolis_torque(q, qd)
            if cfg["grav"]:
                tau = tau + model.gravity(q)
            tau = np.clip(tau, -tau_max, tau_max)
        qdd = model.forward(q, qd, tau)
        qd = qd + qdd * dt
        q = q + qd * dt
        if k % 20 == 0:
            errs.append(np.linalg.norm(chain.tcp(q) - chain.tcp(qr)))
            taus.append(np.abs(tau).max())
            ts.append(t)
    return np.array(errs), np.array(taus), np.array(ts)


def main():
    chain, model = load_dyn()
    mask, grid = rp.ros_logo_mask(LOGO_WIDTH_MM)
    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)

    # The window sits on the R, not at the start of the path.
    #
    # The path begins with the nine dots, which are convex blobs the arm fills
    # in one pass each. The letters are where the machine is actually stressed:
    # every scanline that meets the counter of the R has to lift and re-enter,
    # so that is where the entry transients are. A control study run on the
    # dots would be measuring the easy part of the drawing.
    start = window_start(P_mm, kind, t)
    stop = int(np.searchsorted(t, t[start] + WINDOW_S))
    win = slice(start, stop)
    tt = t[win] - t[start]
    entries = int(((kind[win][:-1] != rp.KIND_MARK) & (kind[win][1:] == rp.KIND_MARK)).sum())
    print(f"window: {tt[-1]:.1f} s from t={t[start]:.0f} s, "
          f"{stop - start} path points, {entries} needle entries", flush=True)

    Qp, conv, _ = solve_path(chain, Pw[win])
    # reference velocity and acceleration by finite difference on the retimed path
    QD = np.gradient(Qp, tt, axis=0)
    QDD = np.gradient(QD, tt, axis=0)

    def stats(e, tau, ts, wn):
        """Split by needle state; only the marking figures are about the drawing.

        A run that leaves the finite numbers is recorded as diverged rather than
        written out as NaN, which is not valid JSON and reads downstream as a
        missing measurement instead of the result it actually is.
        """
        if not (np.isfinite(e).all() and np.isfinite(tau).all()):
            return dict(diverged=True)
        down, settled, _ = needle_state(tt, kind[win], ts, wn)
        return dict(
            diverged=False,
            marking_settled_mean_um=float(e[settled].mean() * 1e6),
            marking_settled_max_um=float(e[settled].max() * 1e6),
            marking_mean_um=float(e[down].mean() * 1e6),
            marking_max_um=float(e[down].max() * 1e6),
            travel_mean_um=float(e[~down].mean() * 1e6),
            tau_peak=float(tau.max()))

    hdr = (f"  {'':16} {'settled':>9} {'set max':>9} {'marking':>9} "
           f"{'mark max':>9} {'travel':>9} {'tau pk':>7}")

    def show(label, r):
        if r["diverged"]:
            print(f"  {label:16} {'diverged':>9}", flush=True)
            return
        print(f"  {label:16} {r['marking_settled_mean_um']:9.1f} "
              f"{r['marking_settled_max_um']:9.1f} {r['marking_mean_um']:9.1f} "
              f"{r['marking_max_um']:9.1f} {r['travel_mean_um']:9.1f} "
              f"{r['tau_peak']:7.2f}", flush=True)

    print(hdr, flush=True)
    results = {}
    for mode in MODES:
        e, tau, ts = run(model, chain, tt, Qp, QD, QDD, wn=20.0, mode=mode)
        results[mode] = stats(e, tau, ts, 20.0)
        show(mode, results[mode])
        if np.isfinite(e).all():
            np.save(OUT / f"err_{mode.replace(' ', '_').replace('+', '_')}.npy", e)

    print(hdr, flush=True)
    sweep = {}
    for wn in (10.0, 20.0, 40.0, 80.0, 120.0):
        e, tau, ts = run(model, chain, tt, Qp, QD, QDD, wn=wn, mode="pid+g")
        sweep[wn] = stats(e, tau, ts, wn)
        show(f"wn={wn:.0f}", sweep[wn])

    # The recommended configuration, measured the same way as everything else.
    # These used to live in a hand-written control_final.json that no script
    # produced, which is the one thing section 11 of the notebook promises never
    # happens.
    print(hdr, flush=True)
    final = {}
    for wn, mode in ((20.0, "pid+g+v"), (40.0, "pid+g"), (40.0, "pid+g+v")):
        e, tau, ts = run(model, chain, tt, Qp, QD, QDD, wn=wn, mode=mode)
        key = f"wn{wn:.0f}_{mode}"
        final[key] = stats(e, tau, ts, wn)
        show(key, final[key])
    (OUT / "control_final.json").write_text(json.dumps(
        {"configs": final, "window_s": float(tt[-1]),
         "window_start_s": float(t[start]), "window_entries": entries}, indent=2))

    (OUT / "control_study.json").write_text(json.dumps(
        {"modes": results, "sweep": {str(k): v for k, v in sweep.items()},
         "window_s": float(tt[-1]), "window_start_s": float(t[start]),
         "window_entries": entries, "logo_width_mm": LOGO_WIDTH_MM}, indent=2))
    print("\nwrote", OUT / "control_study.json")


if __name__ == "__main__":
    main()
