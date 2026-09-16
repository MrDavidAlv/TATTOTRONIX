"""What the path resampling step actually buys.

The error budget listed "0.6 mm between points" against a 0.3 mm line and called
it twice the line width. That conflates the *spacing* of the points with the
*error* they introduce, which is not the same thing: the controller interpolates
between them, so what the spacing costs is chord deviation - how far the straight
run between two points departs from the curve they were sampled from - and that
goes as roughly h^2 / 8R, not as h.

This measures both halves, geometric and dynamic, over the same window as the
other control studies.

    python3 docs/scripts/resample_study.py      # ~10 min

Writes data/resample_study.json.
"""

import json
from pathlib import Path

import numpy as np
from scipy import ndimage

import rospath as rp
from analysis import (LOGO_WIDTH_MM, needle_state, path_to_world, solve_path,
                      time_parameterise)
from control_study import WINDOW_S, run, window_start
from dynamics import load as load_dyn

OUT = Path(__file__).resolve().parents[1] / "data"

STEPS = (0.6, 0.3, 0.15)
WN = 40.0
MODE = "pid+g+v"


def chord_error(mask, grid, step):
    """Worst distance from a traced contour to the chords that resample it.

    Only contours curve; the fill runs in straight lines, where resampling adds
    nothing. So the worst case over the contours is the worst case overall.
    """
    xs, ys = grid
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    res = abs(xs[1] - xs[0])
    worst = 0.0
    for k in range(1, n + 1):
        blob = lab == k
        if blob.sum() * res * res < rp.MIN_BLOB_MM2:
            continue
        for loop in rp._loops(blob):
            rr = np.fromiter((p[0] for p in loop), int, len(loop))
            cc = np.fromiter((p[1] for p in loop), int, len(loop))
            dense = rp._smooth_closed(np.column_stack([xs[cc], ys[rr]]))
            dense = np.vstack([dense, dense[:1]])
            d = np.linalg.norm(np.diff(dense, axis=0), axis=1)
            s = np.concatenate([[0.0], np.cumsum(d)])
            if s[-1] < step:
                continue
            t = np.arange(0.0, s[-1], step)
            for a, b in zip(t[:-1], t[1:]):
                seg = dense[np.searchsorted(s, a):np.searchsorted(s, b) + 1]
                if len(seg) < 3:
                    continue
                v = seg[-1] - seg[0]
                L = np.linalg.norm(v)
                if L < 1e-9:
                    continue
                worst = max(worst, float((np.abs(np.cross(v, seg - seg[0])) / L).max()))
    return worst * 1000.0


def build(step, entry_no=None):
    """Path and window for one resampling step, anchored on a needle entry."""
    old = rp.POINT_STEP
    rp.POINT_STEP = step
    try:
        mask, grid = rp.ros_logo_mask(LOGO_WIDTH_MM)
        P_mm, kind = rp.toolpath(mask, grid)
    finally:
        rp.POINT_STEP = old
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    ent = np.flatnonzero((kind[:-1] != rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK))
    if entry_no is None:
        entry_no = int(np.searchsorted(ent, window_start(P_mm, kind, t)))
    start = int(ent[min(entry_no, len(ent) - 1)])
    stop = int(np.searchsorted(t, t[start] + WINDOW_S))
    return mask, grid, P_mm, kind, Pw, t, slice(start, stop), entry_no


def main():
    chain, model = load_dyn()
    *_, entry_no = build(STEPS[0])
    print(f"window: from needle entry #{entry_no}", flush=True)
    print(f"  {'step':>6} {'points':>8} {'chord':>8} {'settled':>9} {'worst':>9} "
          f"{'tau pk':>7}", flush=True)

    results = {}
    for step in STEPS:
        mask, grid, P_mm, kind, Pw, t, win, _ = build(step, entry_no)
        tt = t[win] - t[win][0]
        Qp, _, _ = solve_path(chain, Pw[win])
        QD = np.gradient(Qp, tt, axis=0)
        QDD = np.gradient(QD, tt, axis=0)
        e, tau, ts = run(model, chain, tt, Qp, QD, QDD, wn=WN, mode=MODE)
        _, settled, _ = needle_state(tt, kind[win], ts, WN)
        down = (kind[win][:-1] == rp.KIND_MARK) & (kind[win][1:] == rp.KIND_MARK)
        dn = np.interp(ts, tt[:-1], down.astype(float)) > 0.5
        results[step] = dict(
            points=int(len(P_mm)),
            chord_um=chord_error(mask, grid, step),
            settled_um=float(e[settled].mean() * 1e6),
            worst_um=float(e[dn].max() * 1e6),
            tau_peak=float(tau.max()))
        r = results[step]
        print(f"  {step:6.2f} {r['points']:8d} {r['chord_um']:8.1f} "
              f"{r['settled_um']:9.1f} {r['worst_um']:9.1f} {r['tau_peak']:7.2f}",
              flush=True)

    (OUT / "resample_study.json").write_text(json.dumps(
        {"step_mm": {str(k): v for k, v in results.items()},
         "wn": WN, "mode": MODE, "window_s": WINDOW_S,
         "window_entry_no": entry_no,
         "mask_res_mm": 0.25}, indent=2))
    print("\nwrote", OUT / "resample_study.json")


if __name__ == "__main__":
    main()
