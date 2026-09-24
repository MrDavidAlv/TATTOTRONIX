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
Check that every launch file still builds a launch description.

The same checks tattotronix_gazebo runs on its own: a launch file is code, and
an argument nobody can discover is one nobody will set.
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
LAUNCH_FILES = sorted(LAUNCH_DIR.glob('*.launch.py'))


def _load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_there_are_launch_files():
    assert LAUNCH_FILES


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda p: p.name)
def test_generates_a_launch_description(path):
    assert isinstance(_load(path).generate_launch_description(), LaunchDescription)


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda p: p.name)
def test_every_argument_is_documented(path):
    entities = _load(path).generate_launch_description().entities
    undocumented = [e.name for e in entities
                    if isinstance(e, DeclareLaunchArgument) and not e.description]
    assert not undocumented, f'arguments without a description: {undocumented}'
