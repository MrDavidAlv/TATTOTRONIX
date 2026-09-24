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
Rigid body dynamics, against an independent route to the same answer.

Gravity torque from the recursive Newton-Euler pass is compared with the
numerical gradient of potential energy. Those are two unrelated derivations - a
rigid body recursion against the derivative of a scalar - so agreeing to machine
precision is evidence, not a tautology. docs/mathematical-model/control.md
quotes this check at three poses.
"""

import numpy as np
import pytest

G = 9.80665
POSES = [
    np.zeros(5),
    np.array([0.3, -0.5, 0.7, 0.2, -0.4]),
    np.array([-0.8, 1.0, -0.6, 0.9, 1.1]),
]


def _potential(model, q):
    """Potential energy of every lumped body, in the base frame."""
    import kinematics
    total = 0.0
    T = np.eye(4)
    for i, b in enumerate(model.bodies):
        rot = kinematics.axis_rotation(b['axis'], q[i])
        T = T @ b['pre'] @ np.block([[rot, np.zeros((3, 1))],
                                     [np.zeros((1, 3)), np.ones((1, 1))]])
        com_world = (T @ np.append(b['com'], 1.0))[:3]
        total += b['m'] * G * com_world[2]
    return total


@pytest.mark.parametrize('q', POSES, ids=['zero', 'pose_a', 'pose_b'])
def test_gravity_matches_the_potential_gradient(chain_and_model, q):
    _, model = chain_and_model
    analytic = model.gravity(q)
    step = 1e-6
    numeric = np.array([
        (_potential(model, q + np.eye(5)[i] * step)
         - _potential(model, q - np.eye(5)[i] * step)) / (2 * step)
        for i in range(5)])
    worst = np.abs(analytic - numeric).max()
    assert worst < 1e-8, f'gravity disagrees with dU/dq by {worst:.2e} N m'


def test_inertia_is_symmetric_and_positive_definite(chain_and_model):
    """Both are properties of a real inertia matrix, and neither is enforced.

    M is recovered from unit-acceleration responses rather than assembled
    directly, so nothing in the construction guarantees either one.
    """
    _, model = chain_and_model
    M = model.inertia(POSES[1])
    assert np.allclose(M, M.T, atol=1e-9), 'inertia matrix is not symmetric'
    assert np.linalg.eigvalsh(M).min() > 0, 'inertia matrix is not positive definite'


def test_joint_two_and_three_differ_by_the_upper_arm_alone(chain_and_model):
    """A result that looks like a bug and is not.

    At the zero pose the joint_2 and joint_3 axes are parallel and sit at the
    same x. Everything beyond joint_3 therefore has the same lever arm about
    both, and cancels out of the difference: the only load joint_2 carries
    that joint_3 does not is the upper arm itself. So the two gravity torques
    differ by exactly the upper arm's own moment, -g m x, with x its centre of
    mass measured from the joint_2 axis.

    This used to be written as "the two torques are equal", which was only
    true because the box approximation put the upper arm's centre of mass
    exactly over the axis. The version that holds for any mass model is the
    difference, and it is checked against a hand computation rather than
    against the torque routine a second time.
    """
    _, model = chain_and_model
    tau = model.gravity(np.zeros(5))
    upper_arm = model.bodies[1]          # the body joint_2 moves
    expected = -G * upper_arm['m'] * upper_arm['com'][0]
    assert tau[1] - tau[2] == pytest.approx(expected, rel=1e-9, abs=1e-15), (
        f'{tau[1]:.9f} - {tau[2]:.9f} N m, expected {expected:.3e}')


def _summary():
    import json
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / 'docs' / 'data' / 'summary.json'
    return json.loads(path.read_text(encoding='utf-8'))


def test_the_published_gains_belong_to_this_description(chain_and_model):
    """The gains in the documents were tuned against the arm as it stands.

    Every gain is pole placement on an effective inertia, and that inertia
    comes from the description's masses. Change the masses without re-running
    the analysis and the documents go on quoting gains, torques and tracking
    errors for a robot that no longer exists - with every other check green,
    because the documents still agree with the data files. They just no longer
    agree with the robot. This is the check that closes that gap.
    """
    s = _summary()
    _, model = chain_and_model
    q_ref = np.array(s['q_ref_tune'])
    now = 1.0 / np.diag(np.linalg.inv(model.inertia(q_ref)))
    assert np.allclose(now, s['J_eff'], rtol=1e-9), (
        'the description\'s inertia no longer matches the one the published '
        'gains were tuned against. Re-run docs/scripts/analysis.py and the '
        f'studies after it.\n  tuned against {np.array(s["J_eff"])}\n  now {now}'
    )


def test_the_published_gains_follow_from_the_published_inertia():
    """Kp = 3 J wn^2, Kd = 3 J wn, Ki = J wn^3, to the rounding they are written at."""
    s = _summary()
    J, wn = np.array(s['J_eff']), s['tune_wn']
    assert np.allclose(s['Kp'], 3 * J * wn ** 2, rtol=0, atol=0.5e-4 + 1e-12)
    assert np.allclose(s['Kd'], 3 * J * wn, rtol=0, atol=0.5e-5 + 1e-12)
    assert np.allclose(s['Ki'], J * wn ** 3, rtol=0, atol=0.5e-4 + 1e-12)


def test_the_published_zero_pose_gravity_is_this_description_s(chain_and_model):
    """control.md quotes it; the value it quotes has to be the arm's."""
    _, model = chain_and_model
    assert np.allclose(model.gravity(np.zeros(5)), _summary()['grav_zero_Nm'],
                       rtol=1e-9, atol=1e-12)
