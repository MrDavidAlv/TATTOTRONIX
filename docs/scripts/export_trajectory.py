# Copyright 2026 Mario David Alvarez Vallejo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Export the solved logo trajectory for the runtime package.

The analysis solves the whole chain offline: artwork to mask to toolpath to
inverse kinematics. Nothing fed that result to the controller, so the arm in
Gazebo had never actually drawn anything. This writes it where a node can pick
it up.

Deliberately a generated artifact with its provenance recorded in the file
header, not a hand-edited table: the trajectory is a *result*, and a result
someone can edit by hand is a result nobody can check.

    python3 docs/scripts/export_trajectory.py                     # the ROS logo
    python3 docs/scripts/export_trajectory.py --image logo.png --name mylogo
    python3 docs/scripts/export_trajectory.py --image photo.jpg --name me \
            --method edges

Writes into src/tattotronix_control/config/trajectories/, one file per drawing,
holding the joint angles, the time at each point, the needle state, and the tip
position the drawing trace is rendered from. `draw` picks one by name:

    ros2 launch tattotronix_gazebo draw.launch.py art:=mylogo
"""

import argparse

from pathlib import Path

import numpy as np

import rospath as rp
from analysis import (ARTWORK_WIDTH_MM, path_to_world, solve_path,
                      time_parameterise)
from dynamics import load as load_dyn

REPO = Path(__file__).resolve().parents[2]
OUT_DIR = REPO / "src/tattotronix_control/config/trajectories"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="any raster image; omit for the ROS logo")
    ap.add_argument("--name", help="output name; defaults to ros_logo or the "
                                   "image's own stem")
    ap.add_argument("--method", default="threshold", choices=("threshold", "edges"),
                    help="threshold for solid artwork, edges for photographs")
    ap.add_argument("--width", type=float, default=ARTWORK_WIDTH_MM,
                    help="how wide the mark is drawn, mm")
    ap.add_argument("--pitch", type=float, default=None,
                    help="fill spacing, mm; defaults to rospath.STROKE_PITCH")
    args = ap.parse_args()

    if args.pitch is not None:
        rp.STROKE_PITCH = args.pitch
    name = args.name or (Path(args.image).stem if args.image else "ros_logo")

    chain, _ = load_dyn()

    if args.image:
        mask, grid = rp.from_image(args.image, args.width, method=args.method)
        source = f"{Path(args.image).name} ({args.method}, {rp.STROKE_PITCH:g} mm pitch)"
    else:
        mask, grid = rp.ros_logo_mask(args.width)
        source = f"official ROS logo ({rp.STROKE_PITCH:g} mm pitch)"

    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    print(f"{source}\n  toolpath: {len(Pw)} points, {t[-1]:.0f} s "
          f"({t[-1] / 60:.1f} min at the planned feed)", flush=True)

    Q, conv, res = solve_path(chain, Pw)
    if not conv.all():
        raise SystemExit(f"  inverse kinematics failed at {(~conv).sum()} points; "
                         "not exporting a trajectory the arm cannot follow")
    print(f"  ik: 100% converged, worst residual {res.max():.2e}", flush=True)

    tcp = np.array([chain.tcp(q) for q in Q])
    marked = np.concatenate([[False],
                             (kind[:-1] == rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK)])

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"{name}.npz"
    np.savez_compressed(
        out,
        joints=np.array(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]),
        q=Q.astype(np.float32),
        t=t.astype(np.float32),
        kind=kind.astype(np.int8),
        marked=marked,
        tcp=tcp.astype(np.float32),
        artwork_width_mm=args.width,
        source=source)
    entries = int(((kind[:-1] != rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK)).sum())
    ink, air = rp.lengths(P_mm, kind)
    print(f"  wrote {out} ({out.stat().st_size / 1024:.0f} KiB)")
    print(f"  {len(Q)} points, {ink:.0f} mm marked, {air:.0f} mm travelled, "
          f"{entries} needle entries")
    print(f"  run it:  ros2 launch tattotronix_gazebo draw.launch.py art:={name}")


if __name__ == "__main__":
    main()
