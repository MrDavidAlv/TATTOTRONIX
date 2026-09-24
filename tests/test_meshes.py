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
The collision geometry has to be the same object as the visual geometry.

A URDF never checks this. It names two files per link and believes both, so a
collision shape can sit rotated ninety degrees from the thing that is drawn and
nothing complains: RViz and Gazebo render the STL and look correct, while every
collision query reads the DAE and answers about a different arm.

That is not hypothetical here. The DAEs carried a node transform that a loader
applies on top of vertices which had already been corrected, so the transform
landed twice, and the arm was in self-collision at the home pose - found only
when MoveIt refused to plan. These tests are what should have caught it, and
they compare the geometry a loader actually sees, node transform included.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'tools'))

MESHES = REPO / 'src' / 'tattotronix_description' / 'meshes'
VISUAL = MESHES / 'visual'
COLLISION = MESHES / 'collision'

#: Mesh units are CAD centimetres. The DAEs store fewer significant digits than
#: the STLs, so an exact comparison rejects a correct file; 0.02 cm is 0.2 mm,
#: the same tolerance the alignment tool matches centroids at.
TOLERANCE_CM = 0.02


def _pairs():
    """Every link that has both geometries, which is what can be compared."""
    return [(p, COLLISION / (p.stem + '.dae')) for p in sorted(VISUAL.glob('*.stl'))
            if (COLLISION / (p.stem + '.dae')).exists()]


def _bounds(points):
    return ([min(p[i] for p in points) for i in range(3)],
            [max(p[i] for p in points) for i in range(3)])


def _apply(matrix, point):
    """A 4x4 row-major COLLADA transform applied to a point."""
    return [sum(matrix[r * 4 + c] * point[c] for c in range(3)) + matrix[r * 4 + 3]
            for r in range(3)]


@pytest.mark.parametrize('stl,dae', _pairs(), ids=lambda p: p.stem)
def test_the_node_transform_is_the_identity(stl, dae):
    """The vertices are in the link frame, so nothing may transform them again.

    This is the specific defect: a loader multiplies the node matrix into the
    vertices, and the alignment tool had already baked that same rotation in.
    """
    import align_collision_meshes as tool
    element, values = tool.geometry_node_matrix(ET.parse(dae).getroot())
    assert element is not None, f'{dae.name} has no node holding the geometry'
    assert tool.node_transform_is_identity(values), (
        f'{dae.name} still carries a node transform, which a loader applies on '
        f'top of vertices that are already correct: {values}. '
        'Run: python3 tools/align_collision_meshes.py --apply'
    )


@pytest.mark.parametrize('stl,dae', _pairs(), ids=lambda p: p.stem)
def test_collision_and_visual_describe_the_same_shape(stl, dae):
    """What a loader sees for collision must match what it draws.

    Compared through the node transform rather than the raw vertices, because
    the raw vertices were already identical while the shapes were not.
    """
    import align_collision_meshes as tool
    _, values = tool.geometry_node_matrix(ET.parse(dae).getroot())
    _, _, _, dae_triangles = tool.read_dae(dae)

    seen = [_apply(values, v) for t in dae_triangles for v in t]
    drawn = [v for t in tool.read_stl(stl) for v in t]
    assert len(seen) == len(drawn), (
        f'{dae.name} has {len(seen) // 3} triangles, {stl.name} has {len(drawn) // 3}'
    )

    lo_s, hi_s = _bounds(seen)
    lo_d, hi_d = _bounds(drawn)
    for axis, name in enumerate('xyz'):
        assert abs(lo_s[axis] - lo_d[axis]) < TOLERANCE_CM, (
            f'{stl.stem}: collision {name} starts at {lo_s[axis]:.3f} cm, '
            f'visual at {lo_d[axis]:.3f} cm'
        )
        assert abs(hi_s[axis] - hi_d[axis]) < TOLERANCE_CM, (
            f'{stl.stem}: collision {name} ends at {hi_s[axis]:.3f} cm, '
            f'visual at {hi_d[axis]:.3f} cm'
        )


@pytest.mark.parametrize('stl,dae', _pairs(), ids=lambda p: p.stem)
def test_the_alignment_tool_has_nothing_left_to_do(stl, dae):
    """Idempotence, which is the whole claim the tool makes about itself."""
    import align_collision_meshes as tool
    stl_triangles = tool.read_stl(stl)
    _, _, _, dae_triangles = tool.read_dae(dae)
    solution = tool.solve(stl_triangles, dae_triangles)
    assert solution is not None, f'{dae.name} is not a rigid transform of {stl.name}'
    matrix, translation = solution
    assert tool.describe(matrix) == '(+x, +y, +z)', f'{dae.name} is still rotated'
    assert all(abs(t) < 1e-4 for t in translation), f'{dae.name} is still translated'
