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
The actuator study: its error model, and the figures it publishes.

The study's headline is a worst case taken over the corners of a box of joint
errors. That shortcut is only right because the tip error is convex in the
joint errors, so it is checked against the inside of the box rather than
trusted.
"""

import json

import numpy as np

import pytest


@pytest.fixture(scope='module')
def act():
    import actuator_study
    return actuator_study


def test_a_joint_rests_within_half_its_step_and_half_its_dead_band(act):
    e = act.hobby(act.PCA9685_STEP_US, {'joint_1': 'large', 'joint_5': 'small'},
                  ['joint_1', 'joint_5'])
    step = 1e6 / 50 / 4096          # a 50 Hz frame in 4096 counts, us
    per_deg = 2000 / 180            # 500 to 2500 us over 180 degrees
    assert e[0] == pytest.approx((step + 5.0) / 2 / per_deg)      # MG996R, 5 us band
    assert e[1] == pytest.approx((step + 10.0) / 2 / per_deg)     # SG90, 10 us band


def test_the_worst_corner_is_the_worst_point_of_the_box(act):
    rng = np.random.default_rng(3)
    J = rng.normal(size=(20, 2, 5))
    e = rng.uniform(0.1, 1.0, size=5)
    worst = act.lateral_worst(J, e)
    inside = rng.uniform(-1, 1, size=(4000, 5)) * e
    sampled = np.linalg.norm(np.einsum('pij,sj->psi', J, inside), axis=2).max(axis=1)
    assert (sampled <= worst * (1 + 1e-12)).all()
    # and a corner attains it: the worst case is reached, not merely a bound
    corners = np.array(np.meshgrid(*[[-1.0, 1.0]] * 5)).reshape(5, -1).T * e
    at_corners = np.linalg.norm(np.einsum('pij,cj->pci', J, corners), axis=2).max(axis=1)
    assert np.allclose(at_corners, worst, rtol=1e-12)


def test_the_published_actuator_figures_are_what_the_script_computes(act):
    """docs/data/actuator_study.json is regenerated, never edited."""
    def flat(obj, prefix=''):
        if isinstance(obj, dict):
            for k, v in obj.items():
                yield from flat(v, f'{prefix}{k}.')
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                yield from flat(v, f'{prefix}{i}.')
        else:
            yield prefix.rstrip('.'), obj

    have = dict(flat(json.loads((act.OUT / 'actuator_study.json').read_text(encoding='utf-8'))))
    want = dict(flat(json.loads(json.dumps(act.study()))))
    assert have.keys() == want.keys(), set(have) ^ set(want)
    for key, value in want.items():
        if isinstance(value, float):
            assert have[key] == pytest.approx(value, rel=1e-12, abs=1e-15), key
        else:
            assert have[key] == value, key
