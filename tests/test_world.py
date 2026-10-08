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
The studio world, held to the planner and to what drawing needs of it.

The work panel is where the design toolchain and tattotronix_control.plan put
a drawing, so its size and place are theirs. It is drawn but not collided with:
it stands in for skin, which the needle enters, and a rigid panel held the
needle out and dragged the arm (see the note in studio.sdf). What holds the
work up, the bench and the ground, stays solid.
"""

import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'docs' / 'scripts'))

import analysis  # noqa: E402

GAZEBO = REPO / 'src' / 'tattotronix_gazebo'
WORLD = ET.parse(GAZEBO / 'worlds' / 'studio.sdf').getroot()


def model(name):
    found = WORLD.find(f"./world/model[@name='{name}']")
    assert found is not None, name
    return found


def numbers(element, path):
    return [float(v) for v in element.find(path).text.split()]


def test_the_panel_is_where_the_planner_draws():
    panel, bench = model('work_surface'), model('bench')
    w, h, thick = numbers(panel, './link/visual/geometry/box/size')
    x, y, z = numbers(panel, './pose')[:3]
    bench_top = (numbers(bench, './pose')[2]
                 + numbers(bench, './link/collision/geometry/box/size')[2] / 2)
    spawn_z = float(re.search(r"'spawn_z',\s*default_value='([0-9.]+)'",
                              (GAZEBO / 'launch' / 'simulation.launch.py').read_text()).group(1))
    assert (w, h) == pytest.approx((analysis.PANEL_W, analysis.PANEL_H))
    assert (x, y) == pytest.approx((analysis.PANEL_X, 0.0))
    # The arm is bolted to the bench top, so the panel's top is PANEL_Z above its origin.
    assert spawn_z == pytest.approx(bench_top)
    assert z + thick / 2 - spawn_z == pytest.approx(analysis.PANEL_Z)


def test_the_needle_can_enter_the_panel():
    panel = model('work_surface')
    assert panel.find('./link/visual') is not None
    assert panel.find('./link/collision') is None


@pytest.mark.parametrize('name', ['ground_plane', 'bench'])
def test_what_holds_the_work_up_is_solid(name):
    assert model(name).find('./link/collision') is not None
