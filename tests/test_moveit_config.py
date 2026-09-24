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
The MoveIt configuration, checked against the things it restates.

A semantic description is almost entirely a second copy of facts that live in
the URDF and in the controller configuration. Second copies rot. These tests
regenerate the derived files and compare, so a link added to the description
or a controller renamed fails here rather than at the first plan.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import yaml

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / 'src' / 'tattotronix_moveit_config' / 'config'
CONTROLLERS = REPO / 'src' / 'tattotronix_control' / 'config' / 'tattotronix_controllers.yaml'


@pytest.fixture(scope='module')
def gen():
    """The generator module, and the URDF it reads, expanded once."""
    import make_moveit_config as m
    return m, m.urdf_xml()


def test_the_srdf_is_what_the_generator_produces(gen):
    """Regenerating must be a no-op, or the URDF has moved on without it."""
    module, root = gen
    assert (CONFIG / 'tattotronix.srdf').read_text(encoding='utf-8') == module.build(root), (
        'the SRDF and the description disagree. '
        'Run: python3 docs/scripts/make_moveit_config.py'
    )


def test_the_joint_limits_are_what_the_generator_produces(gen):
    """Same, for the limits derived from the measured inertia."""
    module, root = gen
    have = (CONFIG / 'joint_limits.yaml').read_text(encoding='utf-8')
    assert have == module.build_limits(root), (
        'joint_limits.yaml is stale. '
        'Run: python3 docs/scripts/make_moveit_config.py'
    )


def test_the_srdf_only_names_links_and_joints_that_exist(gen):
    """A typo in the SRDF is silent until a plan fails, so it is caught here."""
    _, root = gen
    links = {link.get('name') for link in root.findall('link')}
    joints = {j.get('name') for j in root.findall('joint')}
    srdf = ET.parse(CONFIG / 'tattotronix.srdf').getroot()

    for chain in srdf.iter('chain'):
        assert chain.get('base_link') in links, chain.get('base_link')
        assert chain.get('tip_link') in links, chain.get('tip_link')
    for pair in srdf.iter('disable_collisions'):
        assert pair.get('link1') in links, pair.get('link1')
        assert pair.get('link2') in links, pair.get('link2')
    for state in srdf.iter('group_state'):
        for j in state.findall('joint'):
            assert j.get('name') in joints, j.get('name')


def test_only_rigid_pairs_are_disabled(gen):
    """The allowed-collision matrix is the one place a mistake is dangerous.

    Disabling a pair that can actually move into each other does not slow the
    planner down, it lets the arm pass through itself. Nothing may be disabled
    that the generator cannot prove is rigidly attached.
    """
    module, root = gen
    srdf = ET.parse(CONFIG / 'tattotronix.srdf').getroot()
    disabled = {tuple(sorted((p.get('link1'), p.get('link2'))))
                for p in srdf.iter('disable_collisions')}
    assert disabled == set(module.rigid_neighbours(root))


def test_moveit_and_the_controller_agree_on_the_controller(gen):
    """MoveIt must send trajectories to a controller that is listening.

    The names and the joint list exist in two files. If they drift, MoveIt
    plans successfully and then hands the result to nothing.
    """
    mi = yaml.safe_load((CONFIG / 'moveit_controllers.yaml').read_text(encoding='utf-8'))
    rc = yaml.safe_load(CONTROLLERS.read_text(encoding='utf-8'))

    names = mi['moveit_simple_controller_manager']['controller_names']
    declared = set(rc['controller_manager']['ros__parameters'])
    for name in names:
        assert name in declared, f'{name} is not a controller in {CONTROLLERS.name}'
        theirs = rc[name]['ros__parameters']['joints']
        mine = mi['moveit_simple_controller_manager'][name]['joints']
        assert mine == theirs, f'{name}: MoveIt has {mine}, the controller has {theirs}'


def test_the_group_covers_every_movable_joint(gen):
    """The chain has to span the arm, not part of it."""
    module, root = gen
    srdf = ET.parse(CONFIG / 'tattotronix.srdf').getroot()
    state = srdf.find('group_state')
    named = [j.get('name') for j in state.findall('joint')]
    assert named == module.group_joints(root)


def test_inverse_kinematics_is_position_only(gen):
    """Five joints cannot reach an arbitrary six-number pose.

    Asking the stock solver for a full pose makes it fail on nearly every goal.
    If this ever flips to false, the failure shows up as an unreliable planner
    rather than as a configuration error, so it is asserted.
    """
    kin = yaml.safe_load((CONFIG / 'kinematics.yaml').read_text(encoding='utf-8'))
    module, root = gen
    assert len(module.group_joints(root)) == 5
    assert kin['arm']['position_only_ik'] is True


def test_the_limits_stay_inside_the_torque_the_joints_have(gen):
    """The derived acceleration has to be affordable, not merely smooth."""
    module, root = gen
    lims = module.joint_limits(root)
    inertia, _, grav_peak = module.effective_inertia()
    derived = yaml.safe_load(module.build_limits(root))['joint_limits']

    for (name, lim), j_eff in zip(lims.items(), inertia):
        accel = derived[name]['max_acceleration']
        torque = j_eff * accel + grav_peak
        assert torque < 0.5 * lim['effort'], (
            f'{name} needs {torque:.2f} N.m of {lim["effort"]} to make its '
            'acceleration limit, which is more than half the joint has'
        )
