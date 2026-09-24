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
The package layout, enforced rather than described.

docs/architecture.md states which layer each package belongs to and which way
dependencies are allowed to point. An architecture nobody checks decays one
convenient import at a time, so it is checked here: the layers come from
LAYERS below, the edges come from the manifests, and any edge that points the
wrong way fails the build.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / 'src'

#: Layer number per package. A package may depend on a strictly lower layer
#: and on nothing else inside this repository. Same-layer edges are rejected
#: too: two packages at one level that need each other are one package.
LAYERS = {
    'tattotronix_description': 0,   # geometry. Depends on no other package here
    'tattotronix_control': 1,       # controllers and the drawing application
    'tattotronix_moveit_config': 1,  # planning. Parallel to control, not above
    'tattotronix_gazebo': 2,        # the simulator, which assembles the rest
}


def _manifests():
    return sorted(SRC.glob('*/package.xml'))


def _depends(manifest):
    """Every package this manifest names, in any kind of depend tag."""
    root = ET.parse(manifest).getroot()
    out = set()
    for tag in ('depend', 'build_depend', 'buildtool_depend', 'exec_depend',
                'run_depend', 'test_depend'):
        out.update(e.text.strip() for e in root.findall(tag) if e.text)
    return out


def test_every_package_has_a_declared_layer():
    """A new package is a layering decision, so it cannot be made silently."""
    found = {ET.parse(m).getroot().findtext('name').strip() for m in _manifests()}
    assert found <= set(LAYERS), (
        f'packages with no layer in LAYERS: {sorted(found - set(LAYERS))}. '
        'Add it to docs/architecture.md and to LAYERS here, in that order.'
    )


@pytest.mark.parametrize('manifest', _manifests(), ids=lambda m: m.parent.name)
def test_dependencies_point_down(manifest):
    """No package depends on its own layer or above. That is what makes the
    graph acyclic without having to search it.
    """
    me = ET.parse(manifest).getroot().findtext('name').strip()
    mine = LAYERS[me]
    for dep in _depends(manifest) & set(LAYERS):
        assert LAYERS[dep] < mine, (
            f'{me} (layer {mine}) depends on {dep} (layer {LAYERS[dep]}). '
            'Dependencies must point strictly downwards.'
        )


@pytest.mark.parametrize('manifest', _manifests(), ids=lambda m: m.parent.name)
def test_the_description_owns_the_geometry(manifest):
    """Meshes and xacro live in one package.

    Two copies of a robot's geometry drift, and the one that drifts is never
    the one being read. Everything else refers to this package for them.
    """
    pkg = manifest.parent
    stray = [p.relative_to(REPO) for p in pkg.rglob('*')
             if p.suffix.lower() in ('.stl', '.dae', '.xacro', '.urdf')]
    if pkg.name == 'tattotronix_description':
        assert stray, 'the description package has lost its geometry'
    else:
        assert not stray, f'{pkg.name} carries geometry that belongs to the description: {stray}'


def test_the_architecture_record_lists_every_package():
    """The document and the tree do not get to disagree."""
    doc = (REPO / 'docs' / 'architecture.md').read_text(encoding='utf-8')
    for name in LAYERS:
        assert name in doc, f'{name} is not in docs/architecture.md'


def test_the_runtime_never_imports_the_design_toolchain():
    """src/ must not depend on docs/scripts, the arrow never reverses.

    docs/architecture.md and the README both state this rule, and the README
    said a test enforced it before one existed. This is that test: no module
    under src/ may import a design-time script by name, or reach for
    docs/scripts on the import path.
    """
    import ast
    design = {p.stem for p in (REPO / 'docs' / 'scripts').glob('*.py')}
    offenders = []
    for path in SRC.rglob('*.py'):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name.split('.')[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module.split('.')[0]]
            for name in names:
                if name in design:
                    offenders.append(f'{path.relative_to(REPO)} imports {name}')
        if 'docs/scripts' in path.read_text(encoding='utf-8').replace('\\\\', '/') and \
                'sys.path' in path.read_text(encoding='utf-8'):
            offenders.append(f'{path.relative_to(REPO)} puts docs/scripts on sys.path')
    assert not offenders, offenders
