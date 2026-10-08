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
Check that each backend includes its own package's launch files, with their arguments.

The include is decided at launch time, so generating the description proves
nothing about it; this runs the decision itself against a launch context.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch'
DRAW = LAUNCH / 'draw.launch.py'
APP = LAUNCH / 'app.launch.py'


def _module(path=DRAW):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _resolve(context, include):
    assert isinstance(include, IncludeLaunchDescription)
    # Loading it resolves the path, and proves the included file is there and loads.
    include.launch_description_source.get_launch_description(context)
    source = include.launch_description_source.location
    passed = {}
    for name, value in include.launch_arguments:
        key = name if isinstance(name, str) else ''.join(n.perform(context) for n in name)
        passed[key] = value if isinstance(value, str) else value.perform(context)
    return source, passed


def _includes(path, **args):
    context = LaunchContext()
    defaults = {'backend': 'gazebo', 'art': 'ros_logo', 'speed': '1.0', 'use_rviz': 'true',
                'dry_run': 'false', 'port': '9090', 'headless': 'false'}
    defaults.update(args)
    for key, value in defaults.items():
        context.launch_configurations[key] = value
    return [_resolve(context, i) for i in _module(path)._include(context)
            if isinstance(i, IncludeLaunchDescription)]


def _decide(**args):
    (only,) = _includes(DRAW, **args)
    return only


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


def test_the_app_gets_the_idle_simulator_and_rosbridge():
    (sim, sim_args), (bridge, bridge_args) = _includes(APP, backend='gazebo', port='9191',
                                                       use_rviz='false', headless='true')
    assert sim.endswith('tattotronix_gazebo/launch/simulation.launch.py'), sim
    assert sim_args == {'use_rviz': 'false', 'headless': 'true'}
    assert bridge.endswith('rosbridge_server/launch/rosbridge_websocket_launch.xml'), bridge
    assert bridge_args == {'port': '9191', 'send_action_goals_in_new_thread': 'true',
                           'call_services_in_new_thread': 'true',
                           'default_call_service_timeout': '5.0'}


def test_the_app_gets_the_draw_server_on_the_backends_clock():
    from launch_ros.actions import Node
    for backend, sim in (('gazebo', True), ('arm', False)):
        context = LaunchContext()
        for key, value in {'backend': backend, 'port': '9090', 'use_rviz': 'false',
                           'headless': 'true', 'dry_run': 'true'}.items():
            context.launch_configurations[key] = value
        nodes = [a for a in _module(APP)._include(context) if isinstance(a, Node)]
        assert [n.node_executable for n in nodes] == ['draw_server']
        # launch_ros keeps each name as a tuple of substitutions; perform them.
        (given,) = nodes[0]._Node__parameters
        params = {''.join(part.perform(context) for part in name): value
                  for name, value in given.items()}
        assert params == {'use_sim_time': sim}


def test_the_app_gets_the_real_arm_without_its_drawing():
    (arm, arm_args), (bridge, _) = _includes(APP, backend='arm', dry_run='true')
    assert arm.endswith('tattotronix_hardware/launch/arm.launch.py'), arm
    assert arm_args == {'draw': 'false', 'dry_run': 'true'}
    assert bridge.endswith('rosbridge_websocket_launch.xml'), bridge


def test_the_app_launch_refuses_an_unknown_backend():
    with pytest.raises(RuntimeError, match='gazebo, arm'):
        _includes(APP, backend='webots')
