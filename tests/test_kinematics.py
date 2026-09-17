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
Kinematics, against values verified outside this model.

The zero-pose tip position was checked against the live TF tree of the running
system, so it is a fixed point the model is not allowed to move without someone
noticing.
"""

import numpy as np

# docs/mathematical-model/kinematics.md quotes these, and check_docs.py keeps
# the document honest about them. This keeps the code honest about them.
TCP_AT_ZERO_MM = np.array([258.96, -9.44, 245.58])


def test_forward_kinematics_at_the_zero_pose(chain_and_model):
    chain, _ = chain_and_model
    tip = chain.tcp(np.zeros(chain.n)) * 1000.0
    assert np.allclose(tip, TCP_AT_ZERO_MM, atol=0.01), \
        f'tip at {tip.round(2)} mm, verified value is {TCP_AT_ZERO_MM}'


def test_five_actuated_joints(chain_and_model):
    chain, _ = chain_and_model
    assert chain.n == 5


def test_task_jacobian_is_square(chain_and_model):
    """Five rows, not six.

    Three for position and two for the pen axis; rotation about the pen axis is
    not a task constraint because the needle is round. A 6xN Jacobian here would
    mean the solver is fighting for a row no joint combination can satisfy.
    """
    chain, _ = chain_and_model
    J = chain.task_jacobian(np.zeros(chain.n))
    assert J.shape == (5, chain.n)


def test_inverse_kinematics_reaches_a_point_on_the_panel(chain_and_model):
    chain, _ = chain_and_model
    target = np.array([0.21, 0.0, 0.005])
    q, converged, residual = chain.ik(target, [0, 0, -1], np.zeros(chain.n))
    assert converged, f'did not converge, residual {residual:.2e}'
    assert np.allclose(chain.tcp(q), target, atol=1e-4)


def test_joint_limits_are_respected_by_the_solver(chain_and_model):
    chain, _ = chain_and_model
    q, _, _ = chain.ik(np.array([0.21, 0.0, 0.005]), [0, 0, -1], np.zeros(chain.n))
    assert np.abs(q).max() <= 1.57
