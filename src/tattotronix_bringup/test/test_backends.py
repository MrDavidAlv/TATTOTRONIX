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
Check that each backend includes its own package's launch file, with its arguments.

The include is decided at launch time, so generating the description proves
nothing about it; this runs the decision itself against a launch context.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import pytest

DRAW = Path(__file__).resolve().parents[1] / 'launch' / 'draw.launch.py'


def _module():
    spec = importlib.util.spec_from_file_location('draw_launch', DRAW)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _decide(**args):
    context = LaunchContext()
    defaults = {'backend': 'gazebo', 'art': 'ros_logo', 'speed': '1.0',
                'use_rviz': 'true', 'dry_run': 'false'}
    defaults.update(args)
    for key, value in defaults.items():
        context.launch_configurations[key] = value
    (include,) = _module()._include(context)
    assert isinstance(include, IncludeLaunchDescription)
    # Loading it resolves the path, and proves the included file is there and loads.
    include.launch_description_source.get_launch_description(context)
    source = include.launch_description_source.location
    passed = {}
    for name, value in include.launch_arguments:
        key = name if isinstance(name, str) else ''.join(n.perform(context) for n in name)
        passed[key] = value if isinstance(value, str) else value.perform(context)
    return source, passed


def test_gazebo_includes_the_simulator_s_draw():
    source, args = _decide(backend='gazebo', art='semillero', speed='2.0', use_rviz='false')
    assert source.endswith('tattotronix_gazebo/launch/draw.launch.py'), source
    assert args == {'art': 'semillero', 'speed': '2.0', 'use_rviz': 'false'}


def test_arm_includes_the_real_arm_and_asks_it_to_draw():
    source, args = _decide(backend='arm', dry_run='true')
    assert source.endswith('tattotronix_hardware/launch/arm.launch.py'), source
    assert args == {'art': 'ros_logo', 'speed': '1.0', 'draw': 'true', 'dry_run': 'true'}


def test_an_unknown_backend_says_which_ones_exist():
    with pytest.raises(RuntimeError, match='gazebo, arm'):
        _decide(backend='webots')
