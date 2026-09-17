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
Check the shipped trajectories, which are data the arm executes.

Every one of these failing means the arm would be handed something it cannot
follow, or would follow into the panel. They are cheap, and none of them needs
a simulator running.
"""

from pathlib import Path

import numpy as np
import pytest
import yaml

HERE = Path(__file__).resolve().parents[1]
TRAJECTORIES = sorted((HERE / 'config' / 'trajectories').glob('*.npz'))
CONTROLLERS = HERE / 'config' / 'tattotronix_controllers.yaml'

# Panel surface in the arm frame, and how far below it the needle is driven.
# A path that leaves this band is either drawing in mid-air or through the
# bench.
PANEL_Z = 0.005
PLUNGE_MM = 1.5
CLEARANCE_MM = 8.0


def test_there_are_trajectories():
    assert TRAJECTORIES, 'no trajectories installed; the package ships none'


@pytest.fixture(params=TRAJECTORIES, ids=lambda p: p.stem)
def traj(request):
    return np.load(request.param, allow_pickle=False)


def test_has_every_field_the_node_reads(traj):
    for key in ('joints', 'q', 't', 'kind', 'marked', 'tcp'):
        assert key in traj, f'missing {key!r}'


def test_joint_names_match_the_controller(traj):
    configured = yaml.safe_load(CONTROLLERS.read_text())
    expected = configured['arm_controller']['ros__parameters']['joints']
    assert [str(j) for j in traj['joints']] == list(expected)


def test_shapes_agree(traj):
    n = len(traj['q'])
    assert traj['q'].shape == (n, len(traj['joints']))
    for key in ('t', 'kind', 'marked'):
        assert len(traj[key]) == n, f'{key} has {len(traj[key])} entries, q has {n}'
    assert traj['tcp'].shape == (n, 3)


def test_time_is_strictly_increasing(traj):
    """A trajectory point that does not advance time is rejected by the controller."""
    dt = np.diff(traj['t'])
    assert (dt > 0).all(), f'{int((dt <= 0).sum())} points do not advance time'


def test_joint_limits(traj):
    """Every axis travels +-1.57 rad; a path outside that cannot be executed."""
    worst = np.abs(traj['q']).max()
    assert worst <= 1.57, f'joint command of {worst:.4f} rad exceeds the +-1.57 limit'


def test_nothing_is_nan(traj):
    for key in ('q', 't', 'tcp'):
        assert np.isfinite(traj[key]).all(), f'{key} contains non-finite values'


def test_tip_stays_in_the_working_band(traj):
    """Between the plunge depth and the clearance height, in the arm frame."""
    z = traj['tcp'][:, 2]
    lo = PANEL_Z - PLUNGE_MM / 1000.0 - 1e-4
    hi = PANEL_Z + CLEARANCE_MM / 1000.0 + 1e-4
    assert z.min() >= lo, f'tip reaches {z.min() * 1000:.2f} mm, below {lo * 1000:.2f} mm'
    assert z.max() <= hi, f'tip reaches {z.max() * 1000:.2f} mm, above {hi * 1000:.2f} mm'


def test_marking_only_happens_at_depth(traj):
    """If the flag says ink, the needle has to be in the work."""
    marked = traj['marked'].astype(bool)
    if not marked.any():
        pytest.skip('nothing marked')
    z = traj['tcp'][marked, 2]
    assert z.max() <= PANEL_Z + 1e-4, \
        f'marked while {(z.max() - PANEL_Z) * 1000:.2f} mm above the surface'


def test_kind_is_one_of_the_three_states(traj):
    assert set(np.unique(traj['kind'])) <= {0, 1, 2}
