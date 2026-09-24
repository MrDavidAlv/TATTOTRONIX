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
The description's hardware backends, expanded the way the launch files expand them.

The real arm's backend takes its channels from a calibration file, so it can go
wrong in ways the others cannot: a joint with no servo, two servos on one
channel, an effort interface a hobby servo cannot report. These expand the
description and read the result back rather than trusting the xacro.
"""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
XACRO = REPO / 'src' / 'tattotronix_description' / 'urdf' / 'tattotronix.urdf.xacro'
CALIBRATION = REPO / 'src' / 'tattotronix_description' / 'config' / 'servo_calibration.yaml'
JOINTS = [f'joint_{i}' for i in range(1, 6)]


def expand(*args):
    return subprocess.run(['xacro', str(XACRO), *args], capture_output=True, text=True)


def system_of(backend):
    out = expand(f'hardware:={backend}', f'servo_calibration:={CALIBRATION}')
    assert out.returncode == 0, out.stderr
    return ET.fromstring(out.stdout).find('ros2_control')


@pytest.mark.parametrize('backend', ['none', 'mock', 'gz', 'pca9685'])
def test_every_backend_expands(backend):
    out = expand(f'hardware:={backend}', f'servo_calibration:={CALIBRATION}')
    assert out.returncode == 0, out.stderr


def test_an_unknown_backend_stops_the_build():
    """Otherwise it would match no branch and give ros2_control nothing to drive."""
    out = expand('hardware:=arduino')
    assert out.returncode != 0
    assert 'hardware must be' in out.stderr


def test_the_real_arm_gets_every_servo_in_the_calibration():
    system = system_of('pca9685')
    assert system.find('hardware/plugin').text == 'tattotronix_hardware/Pca9685ServoSystem'
    assert sorted(j.get('name') for j in system.findall('joint')) == JOINTS
    servos = yaml.safe_load(CALIBRATION.read_text(encoding='utf-8'))['servos']
    channels = []
    for joint in system.findall('joint'):
        name = joint.get('name')
        params = {p.get('name'): p.text for p in joint.findall('param')}
        for key, value in servos[name].items():
            assert float(params[key]) == pytest.approx(float(value)), (name, key)
        channels.append(int(params['channel']))
        if 'channel_b' in params:
            channels.append(int(params['channel_b']))
    assert len(channels) == len(set(channels)), 'two servos on one channel'


def test_the_tool_reaches_the_driver_with_its_calibration():
    """A speed, not an angle: a gpio with a speed interface, not a joint."""
    system = system_of('pca9685')
    tools = system.findall('gpio')
    assert [t.get('name') for t in tools] == ['tool']
    tool = tools[0]
    assert [c.get('name') for c in tool.findall('command_interface')] == ['speed']
    params = {p.get('name'): p.text for p in tool.findall('param')}
    wanted = yaml.safe_load(CALIBRATION.read_text(encoding='utf-8'))['tool']
    for key, value in wanted.items():
        assert float(params[key]) == pytest.approx(float(value)), key
    joints = system.findall('joint')
    servo_channels = {int(p.text) for j in joints for p in j.findall('param')
                      if p.get('name') in ('channel', 'channel_b')}
    assert int(params['channel']) not in servo_channels


def test_the_real_arm_asks_for_no_effort_state():
    """A hobby servo reports nothing back; the driver refuses to invent an effort."""
    for joint in system_of('pca9685').findall('joint'):
        states = [s.get('name') for s in joint.findall('state_interface')]
        assert states == ['position', 'velocity'], joint.get('name')


def test_the_other_backends_keep_their_effort_state():
    for backend in ('mock', 'gz'):
        for joint in system_of(backend).findall('joint'):
            assert 'effort' in [s.get('name') for s in joint.findall('state_interface')]
