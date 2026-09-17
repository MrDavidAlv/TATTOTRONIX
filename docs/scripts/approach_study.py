"""How much slow approach the needle entry actually needs.

The plunge is a travel move: the reference drops at ten times the marking feed
and stops dead on the surface. The loop arrives carrying v / wn of lag, and the
trajectory marks from the first point, so the needle enters at a depth that is
not the one commanded - at the start of every stroke, 98 times per logo.

`rospath.APPROACH` takes the last few millimetres at marking feed instead. This
sweeps it, from nothing to well past useful, with everything else held fixed.

One confound, stated rather than hidden: the window is 12 s of path, and adding
approach slows the path down, so fewer entries fall inside it - seven at
APPROACH = 0, five at 4 mm. That deflates the *mean* for reasons other than the
thing being tested. It does not touch the *worst* figure, which is a maximum over
entries rather than a sum, so the worst column is the one the comparison rests
on.

Held fixed on purpose: the window is chosen once, at APPROACH = 0, and every run
starts at the same *needle entry number*. Letting each run pick its own busiest
window would change the stretch being measured at the same time as the thing
being tested. Anchoring by position instead of by entry number does not work
either - it lands mid-contour and the window ends up holding one entry rather
than the handful the comparison is about. Entry number survives the extra path
point the approach inserts at every plunge.

    python3 docs/scripts/approach_study.py      # ~12 min

Writes data/approach_study.json.
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

APPROACHES = (0.0, 0.5, 1.0, 2.0, 4.0)
WN = 40.0
MODE = "pid+g+v"


def _entries(kind):
    """Indices where the needle goes into the work."""
    return np.flatnonzero((kind[:-1] != rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK))


def build(approach, entry_no=None):
    """Path, window and reference for one approach distance.

    `entry_no` is which needle entry the window starts at. Passing the same one
    to every run keeps the comparison on the same stretch of the drawing.
    """
    old = rp.APPROACH
    rp.APPROACH = approach
    try:
        mask, grid = rp.ros_logo_mask(ARTWORK_WIDTH_MM)
        P_mm, kind = rp.toolpath(mask, grid)
    finally:
        rp.APPROACH = old
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)

    ent = _entries(kind)
    if entry_no is None:
        start = window_start(P_mm, kind, t)
        entry_no = int(np.searchsorted(ent, start))
    start = int(ent[min(entry_no, len(ent) - 1)])
    stop = int(np.searchsorted(t, t[start] + WINDOW_S))
    return P_mm, kind, Pw, t, slice(start, stop), entry_no


def main():
    chain, model = load_dyn()

    # Window chosen once, with no approach, then reused by entry number.
    P0, k0, _, _, w0, entry_no = build(0.0)
    print(f"window: from needle entry #{entry_no}, "
          f"u = {P0[w0, 0].min():.1f} .. {P0[w0, 0].max():.1f} mm on the panel",
          flush=True)

    print(f"  {'approach':>9} {'entries':>8} {'settled':>9} {'worst':>9} "
          f"{'tau pk':>7} {'cycle':>8}", flush=True)
    results = {}
    for a in APPROACHES:
        P_mm, kind, Pw, t, win, _ = build(a, entry_no)
        tt = t[win] - t[win][0]
        Qp, _, _ = solve_path(chain, Pw[win])
        QD = np.gradient(Qp, tt, axis=0)
        QDD = np.gradient(QD, tt, axis=0)
        e, tau, ts = run(model, chain, tt, Qp, QD, QDD, wn=WN, mode=MODE)
        _, settled, _ = needle_state(tt, kind[win], ts, WN)
        down = (kind[win][:-1] == rp.KIND_MARK) & (kind[win][1:] == rp.KIND_MARK)
        dn = np.interp(ts, tt[:-1], down.astype(float)) > 0.5
        entries = int(((kind[win][:-1] != rp.KIND_MARK)
                       & (kind[win][1:] == rp.KIND_MARK)).sum())
        results[a] = dict(
            entries=entries,
            settled_um=float(e[settled].mean() * 1e6),
            worst_um=float(e[dn].max() * 1e6),
            tau_peak=float(tau.max()),
            cycle_s=float(t[-1]))
        r = results[a]
        print(f"  {a:9.1f} {entries:8d} {r['settled_um']:9.1f} {r['worst_um']:9.1f} "
              f"{r['tau_peak']:7.2f} {r['cycle_s']:8.0f}", flush=True)

    (OUT / "approach_study.json").write_text(json.dumps(
        {"approach_mm": {str(k): v for k, v in results.items()},
         "wn": WN, "mode": MODE, "window_entry_no": entry_no,
         "window_u_mm": [float(P0[w0, 0].min()), float(P0[w0, 0].max())],
         "window_s": WINDOW_S}, indent=2))
    print("\nwrote", OUT / "approach_study.json")


if __name__ == "__main__":
    main()
