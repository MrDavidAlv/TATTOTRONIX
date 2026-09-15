"""Export the solved logo trajectory for the runtime package.

The analysis solves the whole chain offline: artwork to mask to toolpath to
inverse kinematics. Nothing fed that result to the controller, so the arm in
Gazebo had never actually drawn anything. This writes it where a node can pick
it up.

Deliberately a generated artifact with its provenance recorded in the file
header, not a hand-edited table: the trajectory is a *result*, and a result
someone can edit by hand is a result nobody can check.

    python3 docs/scripts/export_trajectory.py

Writes src/tattotronix_control/config/logo_trajectory.npz, holding the joint
angles, the time at each point, the needle state, and the tip position the
drawing trace is rendered from.
"""

from pathlib import Path

import numpy as np

import rospath as rp
from analysis import (LOGO_WIDTH_MM, path_to_world, solve_path,
                      time_parameterise)
from dynamics import load as load_dyn

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "src/tattotronix_control/config/logo_trajectory.npz"


def main():
    chain, _ = load_dyn()

    mask, grid = rp.ros_logo_mask(LOGO_WIDTH_MM)
    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    print(f"toolpath: {len(Pw)} points, {t[-1]:.0f} s", flush=True)

    Q, conv, res = solve_path(chain, Pw)
    if not conv.all():
        raise SystemExit(f"inverse kinematics failed at {(~conv).sum()} points; "
                         "not exporting a trajectory the arm cannot follow")
    print(f"ik: 100% converged, worst residual {res.max():.2e}", flush=True)

    tcp = np.array([chain.tcp(q) for q in Q])
    marked = np.concatenate([[False],
                             (kind[:-1] == rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK)])

    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        OUT,
        joints=np.array(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]),
        q=Q.astype(np.float32),
        t=t.astype(np.float32),
        kind=kind.astype(np.int8),
        marked=marked,
        tcp=tcp.astype(np.float32),
        logo_width_mm=LOGO_WIDTH_MM,
        source="docs/scripts/export_trajectory.py")
    size = OUT.stat().st_size / 1024
    print(f"wrote {OUT} ({size:.0f} KiB)")
    print(f"  {len(Q)} points, {t[-1]:.0f} s, "
          f"{int(((kind[:-1] != rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK)).sum())} needle entries")


if __name__ == "__main__":
    main()
