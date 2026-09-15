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
from analysis import (CONTROL_HZ, PANEL_Z, path_to_world, time_parameterise,
                      solve_path, tune)
from dynamics import load as load_dyn

OUT = Path(__file__).resolve().parents[1] / "data"

MODES = {
    "pid": dict(grav=False, vff=False, aff=False),
    "pid+g": dict(grav=True, vff=False, aff=False),
    "pid+g+v": dict(grav=True, vff=True, aff=False),
    "computed torque": dict(grav=True, vff=True, aff=True),
}


def run(model, chain, tt, QQ, QD, QDD, wn, mode, dt=1.0 / 1000, tau_max=20.0):
    Kp, Ki, Kd = tune(model, QQ[len(QQ) // 2], wn=wn)
    cfg = MODES[mode]
    n = model.n
    q = QQ[0].copy()
    qd = QD[0].copy()
    ei = np.zeros(n)
    ctrl_every = max(int(round((1.0 / CONTROL_HZ) / dt)), 1)
    tau = np.zeros(n)
    steps = int(tt[-1] / dt)
    errs, taus = [], []
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
    return np.array(errs), np.array(taus)


def main():
    chain, model = load_dyn()
    mask, grid = rp.placeholder_mask()
    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    stop = int(np.searchsorted(t, t[0] + 12.0))
    tt = t[:stop] - t[0]
    print(f"window: {tt[-1]:.1f} s, {stop} path points", flush=True)

    Qp, conv, _ = solve_path(chain, Pw[:stop])
    # reference velocity and acceleration by finite difference on the retimed path
    QD = np.gradient(Qp, tt, axis=0)
    QDD = np.gradient(QD, tt, axis=0)

    results = {}
    for mode in MODES:
        e, tau = run(model, chain, tt, Qp, QD, QDD, wn=20.0, mode=mode)
        results[mode] = dict(mean_um=float(e.mean() * 1e6), max_um=float(e.max() * 1e6),
                             tau_peak=float(tau.max()))
        print(f"  {mode:16} mean {e.mean()*1e6:8.1f} um   max {e.max()*1e6:8.1f} um"
              f"   tau_peak {tau.max():.2f} Nm", flush=True)
        np.save(OUT / f"err_{mode.replace(' ', '_').replace('+', '_')}.npy", e)

    sweep = {}
    for wn in (10.0, 20.0, 40.0, 80.0, 120.0):
        e, tau = run(model, chain, tt, Qp, QD, QDD, wn=wn, mode="pid+g")
        sweep[wn] = dict(mean_um=float(e.mean() * 1e6), max_um=float(e.max() * 1e6),
                         tau_peak=float(tau.max()))
        print(f"  wn={wn:6.1f} rad/s  mean {e.mean()*1e6:8.1f} um   "
              f"max {e.max()*1e6:8.1f} um   tau_peak {tau.max():.2f} Nm", flush=True)

    (OUT / "control_study.json").write_text(json.dumps(
        {"modes": results, "sweep": {str(k): v for k, v in sweep.items()},
         "window_s": float(tt[-1])}, indent=2))
    print("\nwrote", OUT / "control_study.json")


if __name__ == "__main__":
    main()
