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
Strokes on the work panel, planned into a joint trajectory at run time.

The same toolpath rules the design toolchain uses for the ROS logo
(docs/scripts/rospath.py and analysis.py), applied to strokes a client sends:
each stroke is landed on in two stages - fast to APPROACH_MM above the depth,
the rest at the marking feed - drawn at the depth, resampled every
POINT_STEP_MM, and left by lifting to CLEARANCE_MM. Marking and the approach run
at FEED_MARK, travel at FEED_TRAVEL. tests/test_draw_planning.py holds every
constant and every step to the design toolchain's.

A stroke's points are millimetres on the panel from its corner nearest the
arm's origin (tattotronix_interfaces/msg/Stroke).
"""

from dataclasses import dataclass

import numpy as np

# The work panel in the arm's frame: 200 x 140 mm centred at x = 0.21 m, its top
# at z = +5 mm (docs/scripts/analysis.py, and the Gazebo world).
PANEL_X = 0.21
PANEL_Z = 0.005
PANEL_W_MM = 200.0
PANEL_H_MM = 140.0

CLEARANCE_MM = 8.0         # travel height above the surface
PLUNGE_DEPTH_MM = 1.5      # how far below the surface the tool is driven
APPROACH_MM = 4.0          # last part of the plunge, at the marking feed
POINT_STEP_MM = 0.15       # arc length the strokes are resampled to

FEED_MARK = 0.006          # m/s with the tool in the work
FEED_TRAVEL = 0.060        # m/s clear of the work

KIND_TRAVEL, KIND_MARK, KIND_APPROACH = 0, 1, 2

# Where the solver starts, as the design toolchain's solve_path does.
Q_START = np.array([0.0, 0.6, -0.9, 0.0, -1.2])
DOWN = np.array([0.0, 0.0, -1.0])


class PlanError(ValueError):
    """A drawing that cannot be drawn, and why."""


@dataclass
class Plan:
    q: np.ndarray            # (N, 5) joint positions, rad
    t: np.ndarray            # (N,) time from the start, s
    kind: np.ndarray         # (N,) travel, mark or approach
    marked: np.ndarray       # (N,) the segment ending here is ink
    tcp: np.ndarray          # (N, 3) the tool centre point, arm frame, m
    stroke: np.ndarray       # (N,) the stroke each point belongs to, -1 for the way home


def check(strokes):
    """Refuse what cannot be drawn: a stroke too short, or a point off the panel."""
    if not strokes:
        raise PlanError("no strokes")
    for i, s in enumerate(strokes):
        s = np.asarray(s, float)
        if s.ndim != 2 or s.shape[1] != 2:
            raise PlanError(f"stroke {i}: points must be (x_mm, y_mm) pairs")
        if len(s) < 2:
            raise PlanError(f"stroke {i}: fewer than two points")
        off = (s[:, 0] < 0) | (s[:, 0] > PANEL_W_MM) | (s[:, 1] < 0) | (s[:, 1] > PANEL_H_MM)
        if off.any():
            j = int(np.flatnonzero(off)[0])
            raise PlanError(f"stroke {i}, point {j} ({s[j, 0]:.1f}, {s[j, 1]:.1f}) mm "
                            f"is off the {PANEL_W_MM:.0f} x {PANEL_H_MM:.0f} mm panel")


def resample(points, step=POINT_STEP_MM):
    """Constant arc-length resampling of a polyline, as rospath._resample."""
    if len(points) < 2:
        return points
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < step:
        return points[[0, -1]]
    t = np.arange(0.0, s[-1], step)
    return np.column_stack([np.interp(t, s, points[:, i]) for i in range(points.shape[1])])


def toolpath(strokes):
    """
    Plan the tool's path through the strokes.

    Returns (N, 3) points in mm - x and y on the panel, z from its surface - the
    kind of each point, and the stroke each belongs to.
    """
    pts, kinds, which = [], [], []

    def add(p, k, i):
        pts.append(p)
        kinds.append(k)
        which.append(i)

    for i, s in enumerate(strokes):
        seg = resample(np.asarray(s, float))
        x0, y0 = seg[0]
        add([x0, y0, CLEARANCE_MM], KIND_TRAVEL, i)
        add([x0, y0, -PLUNGE_DEPTH_MM + APPROACH_MM], KIND_TRAVEL, i)
        add([x0, y0, -PLUNGE_DEPTH_MM], KIND_APPROACH, i)
        for x, y in seg:
            add([x, y, -PLUNGE_DEPTH_MM], KIND_MARK, i)
        add([seg[-1][0], seg[-1][1], CLEARANCE_MM], KIND_TRAVEL, i)
    P = np.array(pts, float)
    kind = np.array(kinds, int)
    which = np.array(which, int)
    # The plunge lands on the stroke's first point and the stroke repeats it; a
    # zero-length segment takes zero time and poisons every derivative after it.
    keep = np.concatenate([[True], np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-9])
    return P[keep], kind[keep], which[keep]


def to_arm(P_mm):
    """
    Panel millimetres, z from the surface, to the arm's frame in metres.

    The design toolchain's artwork millimetres are centred on the panel; these
    start at its corner, so the centre is subtracted first.
    """
    P = np.empty_like(P_mm, dtype=float)
    P[:, 0] = PANEL_X + (P_mm[:, 0] - PANEL_W_MM / 2) / 1000.0
    P[:, 1] = (P_mm[:, 1] - PANEL_H_MM / 2) / 1000.0
    P[:, 2] = PANEL_Z + P_mm[:, 2] / 1000.0
    return P


def time_parameterise(P, kind, speed=1.0):
    """
    Give the cumulative time at each point.

    The marking feed for marking and the approach, the travel feed otherwise,
    both scaled by `speed`.
    """
    if speed <= 0:
        raise PlanError("speed must be positive")
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    marking = (kind[:-1] == KIND_MARK) & (kind[1:] == KIND_MARK)
    approach = (kind[:-1] == KIND_APPROACH) | (kind[1:] == KIND_APPROACH)
    v = np.where(marking | approach, FEED_MARK, FEED_TRAVEL) * speed
    return np.concatenate([[0.0], np.cumsum(d / v)])


def solve(chain, P, q0=Q_START):
    """IK along the path, each point warm started from the last, tool pointing down."""
    Q = np.zeros((len(P), chain.n))
    conv = np.zeros(len(P), bool)
    res = np.zeros(len(P))
    q = np.array(q0, float)
    for i, p in enumerate(P):
        q, c, r = chain.ik(p, DOWN, q)
        Q[i], conv[i], res[i] = q, c, r
    return Q, conv, res


def plan(chain, strokes, speed=1.0):
    """Plan a drawing as a joint trajectory, or raise PlanError saying why not."""
    check(strokes)
    P_mm, kind, which = toolpath(strokes)
    P = to_arm(P_mm)
    t = time_parameterise(P, kind, speed)
    Q, conv, _ = solve(chain, P)
    if not conv.all():
        i = int(np.flatnonzero(~conv)[0])
        raise PlanError(f"the arm cannot reach stroke {which[i]} at "
                        f"({P_mm[i, 0]:.1f}, {P_mm[i, 1]:.1f}) mm with the tool upright")
    tcp = np.array([chain.tcp(q) for q in Q])
    marked = np.concatenate([[False], (kind[:-1] == KIND_MARK) & (kind[1:] == KIND_MARK)])
    return Plan(q=Q, t=t, kind=kind, marked=marked, tcp=tcp, stroke=which)
