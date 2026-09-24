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
The wiring diagram, held to the calibration it is drawn from.

A drawing of the electronics is only worth publishing if it cannot drift from
the arm: the channel each servo is on comes from servo_calibration.yaml, the
same file the driver reads, and the published SVG has to be what the script
draws from it now.
"""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
CALIBRATION = REPO / 'src' / 'tattotronix_description' / 'config' / 'servo_calibration.yaml'
PUBLISHED = REPO / 'docs' / 'figures' / '31_wiring.svg'


@pytest.fixture(scope='module')
def wd():
    import wiring_diagram
    return wiring_diagram


def test_every_servo_in_the_calibration_is_on_the_drawing(wd):
    cal = yaml.safe_load(CALIBRATION.read_text(encoding='utf-8'))
    wanted = {(s['channel'], j) for j, s in cal['servos'].items()}
    wanted |= {(s['channel_b'], j) for j, s in cal['servos'].items() if 'channel_b' in s}
    wanted.add((cal['tool']['channel'], 'tool'))
    drawn = {(channel, joint) for channel, joint, _ in wd.channels()}
    assert drawn == wanted
    channels = [channel for channel, _, _ in wd.channels()]
    assert len(channels) == len(set(channels)), 'two servos drawn on one channel'


def test_the_published_diagram_is_what_the_script_draws(wd, tmp_path):
    wd.write(tmp_path)
    fresh = (tmp_path / '31_wiring.svg').read_text(encoding='utf-8')
    assert fresh == PUBLISHED.read_text(encoding='utf-8'), (
        'docs/figures/31_wiring.svg is stale. Run: python3 docs/scripts/wiring_diagram.py')
    assert (REPO / 'docs' / 'figures' / '31_wiring.png').exists()
