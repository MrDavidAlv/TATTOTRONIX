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

"""
Every kinematic figure kinematics.md quotes, computed rather than transcribed.

The document had carried numbers from an older, coarser path - "all 4372 path
points" long after the path had become 16 656 - and nothing noticed, because a
count has no unit for the traceability check to read. So they are produced
here and written to docs/data/kinematics_study.json, where check_docs can hold
the document to them.

    python3 docs/scripts/kinematics_study.py        # a few minutes
"""

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import analysis  # noqa: E402
import kinematics  # noqa: E402
import mass_properties  # noqa: E402
import rospath as rp  # noqa: E402

OUT = HERE.parent / "data" / "kinematics_study.json"

#: Drawing width and offset in +x, mm: the placements kinematics.md compares.
SWEEP = [(120, 0), (150, 0), (120, 15), (100, 30), (150, 15), (120, 30),
         (150, 30), (150, 45)]


def _path(width_mm, offset_mm):
    mask, grid = rp.ros_logo_mask(width_mm)
    P_mm, _ = rp.toolpath(mask, grid)
    P = analysis.path_to_world(P_mm)
    P[:, 0] += offset_mm / 1000.0
    return P


def _svd_along(chain, Q):
    sv = np.array([np.linalg.svd(chain.task_jacobian(q), compute_uv=False) for q in Q])
    return sv


def placement(args):
    """Reach, weakest singular value and convergence for one placement."""
    width, offset = args
    chain = kinematics.Chain.from_description()
    Q, conv, _ = analysis.solve_path(chain, _path(width, offset))
    sv = _svd_along(chain, Q[conv])
    return {"width_mm": width, "offset_mm": offset, "reach_mm": width / 2 + offset,
            "sigma5_min": float(sv[:, -1].min()),
            "cond_max": float((sv[:, 0] / sv[:, -1]).max()),
            "ik_converged_pct": float(100 * conv.mean())}


def highest_point_mm(chain):
    """Highest point of the arm's visual meshes at the zero pose."""
    frames = dict(chain.frames(np.zeros(chain.n)))
    top = -np.inf
    for name in mass_properties.DECLARED:
        tris = mass_properties.read_stl(mass_properties.MESHES / (name + ".stl"))
        pts = tris.reshape(-1, 3)
        T = frames[name]
        z = (pts @ T[:3, :3].T + T[:3, 3])[:, 2]
        top = max(top, float(z.max()))
    return top * 1000


def main():
    chain = kinematics.Chain.from_description()
    frames = dict(chain.frames(np.zeros(chain.n)))
    out = {"zero_pose": {
        "tool0_mm": (frames["tool0"][:3, 3] * 1000).tolist(),
        "tcp_mm": (chain.tcp(np.zeros(chain.n)) * 1000).tolist(),
        "wrist_axis_z_mm": float(frames["wrist_link"][2, 3] * 1000),
        "highest_mm": highest_point_mm(chain),
    }}

    P = _path(analysis.ARTWORK_WIDTH_MM, 0.0)
    Q, conv, res = analysis.solve_path(chain, P)
    sv = _svd_along(chain, Q)
    cond = sv[:, 0] / sv[:, -1]
    worst = int(np.argmax(cond))
    _, _, vt = np.linalg.svd(chain.task_jacobian(Q[worst]))
    out["path"] = {
        "points": int(len(P)),
        "ik_converged_pct": float(100 * conv.mean()),
        "ik_worst_residual": float(res.max()),
        "sigma1_min": float(sv[:, 0].min()), "sigma1_max": float(sv[:, 0].max()),
        "sigma5_min": float(sv[:, -1].min()), "sigma5_max": float(sv[:, -1].max()),
        "cond_min": float(cond.min()), "cond_max": float(cond.max()),
        "worst_point_mm": ((P[worst, :2] - [analysis.PANEL_X, 0.0]) * 1000).tolist(),
        "worst_weakest_direction": vt[-1].tolist(),
        "joint4_min": float(Q[:, 3].min()), "joint4_max": float(Q[:, 3].max()),
        "joint4_column_norm_at_worst": float(np.linalg.norm(chain.task_jacobian(Q[worst])[:, 3])),
    }

    with ProcessPoolExecutor() as pool:
        out["placements"] = list(pool.map(placement, SWEEP))

    OUT.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
