"""Does a faster controller buy anything, and how much?

The error budget's largest remaining modelled term is the worst instant of
tracking, and nothing moves it without more closed-loop bandwidth. Bandwidth is
capped by the rate the loop runs at: every sweep so far has diverged above
40 rad/s, and that ceiling is set by the 200 Hz `controller_manager` rate rather
than by the gains.

This sweeps the rate and the bandwidth together to find where the ceiling
actually is, and what the error does once it moves.

The plant is integrated at 4 kHz throughout, not at 1 kHz as elsewhere. At 1 kHz
a commanded 800 Hz and a commanded 1000 Hz both round to "every step", so the
faster rates would silently be the same experiment.

    python3 docs/scripts/rate_study.py      # ~40 min

Writes data/rate_study.json.
"""

import json
from pathlib import Path

import numpy as np

import rospath as rp
from analysis import (ARTWORK_WIDTH_MM, needle_state, path_to_world, solve_path,
                      time_parameterise)
from control_study import WINDOW_S, run, window_start
from dynamics import load as load_dyn

OUT = Path(__file__).resolve().parents[1] / "data"

RATES = (200.0, 500.0, 1000.0)
WNS = (40.0, 80.0, 160.0)
MODE = "pid+g+v"
PLANT_DT = 1.0 / 4000
TAU_MAX = 20.0


def main():
    chain, model = load_dyn()

    mask, grid = rp.ros_logo_mask(ARTWORK_WIDTH_MM)
    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    start = window_start(P_mm, kind, t)
    stop = int(np.searchsorted(t, t[start] + WINDOW_S))
    win = slice(start, stop)
    tt = t[win] - t[start]
    entries = int(((kind[win][:-1] != rp.KIND_MARK)
                   & (kind[win][1:] == rp.KIND_MARK)).sum())
    print(f"window: {tt[-1]:.1f} s from t={t[start]:.0f} s, {entries} needle entries",
          flush=True)

    Qp, _, _ = solve_path(chain, Pw[win])
    QD = np.gradient(Qp, tt, axis=0)
    QDD = np.gradient(QD, tt, axis=0)

    print(f"  {'rate':>7} {'wn':>6} {'settled':>9} {'worst':>10} {'tau pk':>8} "
          f"{'verdict':>10}", flush=True)
    results = {}
    for rate in RATES:
        for wn in WNS:
            e, tau, ts = run(model, chain, tt, Qp, QD, QDD, wn=wn, mode=MODE,
                             dt=PLANT_DT, tau_max=TAU_MAX, control_hz=rate)
            key = f"{rate:.0f}_{wn:.0f}"
            if not (np.isfinite(e).all() and np.isfinite(tau).all()):
                results[key] = dict(rate=rate, wn=wn, verdict="diverged")
                print(f"  {rate:7.0f} {wn:6.0f} {'':>9} {'':>10} {'':>8} "
                      f"{'diverged':>10}", flush=True)
                continue
            _, settled, _ = needle_state(tt, kind[win], ts, wn)
            down = (kind[win][:-1] == rp.KIND_MARK) & (kind[win][1:] == rp.KIND_MARK)
            dn = np.interp(ts, tt[:-1], down.astype(float)) > 0.5
            # Saturating the torque limit is not the same failure as running away
            # to non-finite numbers, and calling both "unstable" hides which one
            # a faster controller would fix.
            saturated = bool(tau.max() >= TAU_MAX * 0.999)
            results[key] = dict(
                rate=rate, wn=wn,
                settled_um=float(e[settled].mean() * 1e6),
                worst_um=float(e[dn].max() * 1e6),
                tau_peak=float(tau.max()),
                verdict="saturated" if saturated else "ok")
            r = results[key]
            print(f"  {rate:7.0f} {wn:6.0f} {r['settled_um']:9.1f} "
                  f"{r['worst_um']:10.1f} {r['tau_peak']:8.2f} {r['verdict']:>10}",
                  flush=True)

    (OUT / "rate_study.json").write_text(json.dumps(
        {"runs": results, "mode": MODE, "plant_dt": PLANT_DT,
         "tau_max": TAU_MAX, "window_s": WINDOW_S, "window_entries": entries},
        indent=2))
    print("\nwrote", OUT / "rate_study.json")


if __name__ == "__main__":
    main()
