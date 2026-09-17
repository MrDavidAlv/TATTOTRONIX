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
Shared fixtures for the repository-wide tests.

These cover docs/scripts, which is not a ROS package and so is not reached by
colcon test. Run them with `pytest tests/` — the container and CI both do.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / 'docs' / 'scripts'
sys.path.insert(0, str(SCRIPTS))


@pytest.fixture(scope='session')
def repo():
    return REPO


@pytest.fixture(scope='session')
def chain_and_model():
    """The arm, parsed from the live URDF.

    Session scoped because building it shells out to xacro, and doing that once
    per test would dominate the run.
    """
    from dynamics import load
    return load()
