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
The run-time drawing planner, held to the design toolchain.

src/ may not import docs/scripts/, so tattotronix_control carries its own
kinematics (arm.py) and toolpath (plan.py) for drawings a client sends. This is
what stops them drifting: their constants, resampling, panel mapping, timing
and inverse kinematics must agree with rospath.py, analysis.py and
kinematics.py on the same URDF.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'docs' / 'scripts'))
sys.path.insert(0, str(REPO / 'src' / 'tattotronix_control'))

import analysis  # noqa: E402
import kinematics  # noqa: E402
import rospath  # noqa: E402
from tattotronix_control import arm, plan  # noqa: E402

URDF = (REPO / 'docs' / 'notebooks' / 'tattotronix.urdf').read_text()


@pytest.fixture(scope='module')
def chains():
    return arm.Chain(URDF), kinematics.Chain(URDF)


def square(x0, y0, side):
    return np.array([[x0, y0], [x0 + side, y0], [x0 + side, y0 + side],
                     [x0, y0 + side], [x0, y0]], float)


def test_the_constants_are_the_design_toolchains():
    assert plan.CLEARANCE_MM == rospath.CLEARANCE
    assert plan.PLUNGE_DEPTH_MM == rospath.PLUNGE_DEPTH
    assert plan.APPROACH_MM == rospath.APPROACH
    assert plan.POINT_STEP_MM == rospath.POINT_STEP
    assert (plan.KIND_TRAVEL, plan.KIND_MARK, plan.KIND_APPROACH) == \
        (rospath.KIND_TRAVEL, rospath.KIND_MARK, rospath.KIND_APPROACH)
    assert plan.FEED_MARK == analysis.FEED_MARK
    assert plan.FEED_TRAVEL == analysis.FEED_TRAVEL
    assert plan.PANEL_X == analysis.PANEL_X
    assert plan.PANEL_Z == analysis.PANEL_Z
    assert (plan.PANEL_W_MM, plan.PANEL_H_MM) == \
        pytest.approx((analysis.PANEL_W * 1000, analysis.PANEL_H * 1000))


def test_resampling_is_rospaths():
    rng = np.random.default_rng(3)
    poly = np.cumsum(rng.uniform(-4, 4, (12, 2)), axis=0) + 100
    np.testing.assert_array_equal(plan.resample(poly), rospath._resample(poly))


def test_the_panel_maps_as_path_to_world_does():
    """Corner-based millimetres, less the panel's centre, are the artwork's."""
    rng = np.random.default_rng(4)
    P = np.column_stack([rng.uniform(0, 200, 50), rng.uniform(0, 140, 50),
                         rng.uniform(-2, 9, 50)])
    centred = P - [plan.PANEL_W_MM / 2, plan.PANEL_H_MM / 2, 0.0]
    np.testing.assert_allclose(plan.to_arm(P), analysis.path_to_world(centred), atol=1e-15)


def test_timing_is_analysiss():
    P_mm, kind, _ = plan.toolpath([square(40, 30, 20), square(120, 80, 10)])
    P = plan.to_arm(P_mm)
    np.testing.assert_array_equal(plan.time_parameterise(P, kind),
                                  analysis.time_parameterise(P, kind))


def test_a_stroke_is_landed_drawn_and_left_as_rospath_does():
    P_mm, kind, which = plan.toolpath([square(40, 30, 20)])
    assert list(kind[:3]) == [plan.KIND_TRAVEL, plan.KIND_TRAVEL, plan.KIND_APPROACH]
    assert P_mm[0, 2] == plan.CLEARANCE_MM
    assert P_mm[1, 2] == -plan.PLUNGE_DEPTH_MM + plan.APPROACH_MM
    assert P_mm[2, 2] == -plan.PLUNGE_DEPTH_MM
    assert np.all(P_mm[kind == plan.KIND_MARK, 2] == -plan.PLUNGE_DEPTH_MM)
    assert kind[-1] == plan.KIND_TRAVEL and P_mm[-1, 2] == plan.CLEARANCE_MM
    assert set(which) == {0}
    steps = np.linalg.norm(np.diff(P_mm[kind == plan.KIND_MARK], axis=0), axis=1)
    assert steps.max() == pytest.approx(plan.POINT_STEP_MM, abs=1e-9)


def test_inverse_kinematics_is_kinematicss(chains):
    run, design = chains
    P = plan.to_arm(np.array([[30.0, 20.0, -1.5], [100.0, 70.0, -1.5], [170.0, 120.0, 8.0],
                              [10.0, 130.0, 2.5]]))
    q_run, q_design = plan.Q_START.copy(), plan.Q_START.copy()
    for p in P:
        q_run, c_run, r_run = run.ik(p, plan.DOWN, q_run)
        q_design, c_design, r_design = design.ik(p, [0, 0, -1], q_design)
        np.testing.assert_allclose(q_run, q_design, atol=1e-12)
        assert c_run == c_design
        assert r_run == pytest.approx(r_design, abs=1e-15)


def test_a_square_is_drawn_where_it_was_asked(chains):
    run, _ = chains
    p = plan.plan(run, [square(80, 50, 20)])
    marked = p.kind == plan.KIND_MARK
    want = plan.to_arm(plan.toolpath([square(80, 50, 20)])[0])
    np.testing.assert_allclose(p.tcp, want, atol=2e-6)
    assert np.all(np.abs(p.tcp[marked, 2] - (plan.PANEL_Z - plan.PLUNGE_DEPTH_MM / 1000)) < 2e-6)
    # The ink is the stroke as rospath resamples it - chords every 0.15 mm, which
    # shave the corners and leave off the last part-step - less its first chord:
    # the landing point is the stroke's first point, the duplicate is dropped as
    # rospath drops it, and that chord is counted as the approach's end, as in
    # every trajectory export_trajectory.py writes.
    chords = np.linalg.norm(np.diff(rospath._resample(square(80, 50, 20)), axis=0), axis=1)
    ink = np.linalg.norm(np.diff(p.tcp, axis=0), axis=1)[p.marked[1:]].sum()
    assert ink == pytest.approx((chords.sum() - chords[0]) / 1000, abs=1e-6)


def test_speed_scales_the_time(chains):
    run, _ = chains
    one = plan.plan(run, [square(80, 50, 10)], speed=1.0)
    two = plan.plan(run, [square(80, 50, 10)], speed=2.0)
    assert two.t[-1] == pytest.approx(one.t[-1] / 2)


def test_a_drawing_too_fast_for_the_joints_is_refused_with_the_speed_that_will_do(chains):
    run, _ = chains
    big = [square(20, 20, 100), square(150, 20, 30)]
    with pytest.raises(plan.PlanError, match='rad/s limit; the fastest this drawing can go is') \
            as refused:
        plan.plan(run, big, speed=20.0)
    fastest = float(str(refused.value).rsplit(' ', 1)[1])
    assert 1.0 < fastest < 20.0
    p = plan.plan(run, big, speed=fastest)
    rate = np.abs(np.diff(p.q, axis=0)) / np.diff(p.t)[:, None]
    assert np.all(rate <= run.velocity)


def test_a_lift_takes_the_needle_straight_up_to_the_travel_height(chains):
    run, _ = chains
    p = plan.plan(run, [square(80, 50, 20)])
    q = p.q[np.flatnonzero(p.kind == plan.KIND_MARK)[40]]
    Q, t = plan.lift(run, q, speed=2.0)
    tcp = np.array([run.tcp(qi) for qi in Q])
    assert np.abs(tcp[:, :2] - tcp[0, :2]).max() < 2e-6
    assert tcp[0, 2] == pytest.approx(plan.PANEL_Z - plan.PLUNGE_DEPTH_MM / 1000, abs=2e-6)
    assert tcp[-1, 2] == pytest.approx(plan.PANEL_Z + plan.CLEARANCE_MM / 1000, abs=2e-6)
    assert np.all(np.diff(tcp[:, 2]) > 0)
    rise = (plan.CLEARANCE_MM + plan.PLUNGE_DEPTH_MM) / 1000
    # The rise starts at the depth the IK reached, so to its tolerance of a micrometre.
    assert t[-1] == pytest.approx(rise / (plan.FEED_TRAVEL * 2.0), abs=1e-5)
    assert plan.lift(run, Q[-1]) is None


def test_planning_reports_its_progress(chains):
    run, _ = chains
    shares = []
    plan.plan(run, [square(20, 20, 100)], report=shares.append)
    assert shares[0] == 0.0 and shares[-1] == 1.0
    assert shares == sorted(shares) and len(shares) > 3


@pytest.mark.parametrize('strokes, says', [
    ([], 'no strokes'),
    ([[[10.0, 10.0]]], 'fewer than two points'),
    ([[[10.0, 10.0], [210.0, 10.0]]], 'off the 200 x 140 mm panel'),
    ([[[10.0, -1.0], [20.0, 10.0]]], 'off the 200 x 140 mm panel'),
])
def test_what_cannot_be_drawn_is_refused(chains, strokes, says):
    run, _ = chains
    with pytest.raises(plan.PlanError, match=says):
        plan.plan(run, strokes)
