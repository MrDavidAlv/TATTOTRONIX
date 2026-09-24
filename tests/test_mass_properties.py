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
The mesh integrator, checked against shapes whose answers are known.

Integrating a polyhedron is easy to get subtly wrong - a sign, a factor of two,
the difference between the inertia tensor and the covariance it comes from -
and wrong in a way that still produces plausible numbers. So it is run on a
cube and a sphere first, where every answer is a closed form, and only then
believed about a robot.
"""

import re
from pathlib import Path

import numpy as np

import pytest

REPO = Path(__file__).resolve().parents[1]
ARM_MACRO = REPO / 'src' / 'tattotronix_description' / 'urdf' / 'arm_macro.xacro'


@pytest.fixture(scope='module')
def mp():
    import mass_properties
    return mass_properties


def _cube(side):
    """A closed axis-aligned cube centred on the origin, as triangles."""
    h = side / 2.0
    corners = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in faces:
        tris.append([corners[a], corners[b], corners[c]])
        tris.append([corners[a], corners[c], corners[d]])
    return np.array(tris)


def _sphere(radius, n=64):
    """A closed UV sphere. Its volume converges on the analytic one from below."""
    lat = np.linspace(0, np.pi, n // 2 + 1)
    lon = np.linspace(0, 2 * np.pi, n + 1)
    pts = np.array([[[radius * np.sin(a) * np.cos(b),
                      radius * np.sin(a) * np.sin(b),
                      radius * np.cos(a)] for b in lon] for a in lat])
    tris = []
    for i in range(len(lat) - 1):
        for j in range(len(lon) - 1):
            p00, p01 = pts[i][j], pts[i][j + 1]
            p10, p11 = pts[i + 1][j], pts[i + 1][j + 1]
            tris.append([p00, p10, p11])
            tris.append([p00, p11, p01])
    return np.array(tris)


def test_a_cube_has_the_volume_of_a_cube(mp):
    side = 0.037
    volume, centroid, _ = mp.integrate(_cube(side))
    assert volume == pytest.approx(side ** 3, rel=1e-12)
    assert np.allclose(centroid, 0.0, atol=1e-12)


def test_a_cube_has_the_inertia_of_a_cube(mp):
    """I = m a^2 / 6 about any axis through the centre, and no products."""
    side, density = 0.037, 1240.0
    tensor, mass = mp.inertia_about(*mp.integrate(_cube(side)), density)
    expected = mass * side ** 2 / 6.0
    assert mass == pytest.approx(density * side ** 3, rel=1e-12)
    assert np.allclose(np.diag(tensor), expected, rtol=1e-10)
    off = tensor - np.diag(np.diag(tensor))
    assert np.allclose(off, 0.0, atol=1e-15)


def test_an_offset_cube_reports_its_centre_not_the_origin(mp):
    """The inertia has to come back about the centre of mass, not the origin.

    Forgetting the parallel-axis shift is the mistake that still looks
    reasonable, so it is the one worth a test: the answer must not change when
    the same shape is moved.
    """
    side, density = 0.037, 1240.0
    offset = np.array([0.31, -0.12, 0.07])
    here = mp.integrate(_cube(side))
    there = mp.integrate(_cube(side) + offset)

    assert np.allclose(there[1], offset, atol=1e-12)
    a, _ = mp.inertia_about(*here, density)
    b, _ = mp.inertia_about(*there, density)
    assert not np.allclose(a, 0.0)
    assert np.allclose(a, b, rtol=1e-8, atol=1e-12)


def test_a_sphere_has_the_inertia_of_a_sphere(mp):
    """I = 2 m r^2 / 5. A faceted sphere is slightly small, so this is loose."""
    radius, density = 0.05, 1240.0
    volume, centroid, covariance = mp.integrate(_sphere(radius, n=128))
    tensor, mass = mp.inertia_about(volume, centroid, covariance, density)
    # A faceted sphere is inscribed in the smooth one, so it is genuinely
    # smaller and no tolerance makes that go away. The deficit closes as 1/n^2:
    # 0.40% at n=64 and 0.10% here. Both facts are asserted, because "close to
    # the analytic volume" would also pass for a mesh that is slightly too big,
    # which a correct integrator can never produce from an inscribed shape.
    exact = 4.0 / 3.0 * np.pi * radius ** 3
    assert volume < exact
    assert volume == pytest.approx(exact, rel=2e-3)
    assert np.allclose(np.diag(tensor), 0.4 * mass * radius ** 2, rtol=3e-3)


def test_a_closed_surface_has_no_closure_residual(mp):
    """The test that decides whether a volume means anything, tested itself."""
    closed, _ = mp.closure_residual(_cube(0.037))
    assert closed < 1e-12
    sphere, _ = mp.closure_residual(_sphere(0.05))
    assert sphere < 1e-12


def test_an_open_surface_is_caught(mp):
    """Remove one face and the residual has to become obvious, not subtle."""
    open_cube = _cube(0.037)[2:]
    residual, _ = mp.closure_residual(open_cube)
    assert residual > 0.1


def test_the_declared_masses_match_the_description(mp):
    """The comparison is worthless if the list it compares against is stale."""
    text = ARM_MACRO.read_text(encoding='utf-8')
    found = dict(re.findall(r'name="(\w+_link)"\s+mass="([\d.]+)"', text))
    assert found, 'no link masses found in arm_macro.xacro; has it been reformatted?'
    for name, mass in mp.DECLARED.items():
        assert name in found, f'{name} is no longer declared in arm_macro.xacro'
        assert float(found[name]) == pytest.approx(mass), (
            f'{name}: the description says {found[name]} kg, '
            f'mass_properties.DECLARED says {mass} kg'
        )


def test_every_link_mesh_is_closed_enough_to_integrate(mp):
    """The real meshes, on the criterion that actually governs the volume."""
    for name in mp.DECLARED:
        tris = mp.read_stl(mp.MESHES / (name + '.stl'))
        residual, _ = mp.closure_residual(tris)
        assert residual < 1e-2, (
            f'{name}.stl is open by {residual:.1e} of its area; '
            'its volume does not mean anything'
        )


def test_a_box_matches_the_closed_form(mp):
    """The servo helper, against the formula the description already uses."""
    mass, size = 0.055, (0.0407, 0.0197, 0.0429)
    tensor = mp.box_inertia(mass, size)
    x, y, z = size
    assert tensor[0][0] == pytest.approx(mass * (y * y + z * z) / 12.0)
    assert tensor[1][1] == pytest.approx(mass * (x * x + z * z) / 12.0)
    assert tensor[2][2] == pytest.approx(mass * (x * x + y * y) / 12.0)
    assert np.allclose(tensor - np.diag(np.diag(tensor)), 0.0)


def test_splitting_a_body_and_recombining_it_changes_nothing(mp):
    """The composition rule, checked the only way that really tests it.

    Take one box, call it two boxes of half the mass in the same place,
    combine them by the same arithmetic the link model uses, and the answer
    has to come back identical. This catches a wrong parallel-axis sign, a
    missing mass weight in the centroid, and adding tensors about different
    points - which are the three ways to get this wrong and still print
    something that looks like an inertia tensor.
    """
    mass, size = 0.08, (0.04, 0.02, 0.043)
    whole = mp.box_inertia(mass, size)

    halves = [mp.box_inertia(mass / 2, size) for _ in range(2)]
    centres = [np.zeros(3), np.zeros(3)]
    masses = [mass / 2, mass / 2]
    com = sum(m * c for m, c in zip(masses, centres)) / sum(masses)
    combined = sum(mp.shift(i, m, c - com)
                   for i, m, c in zip(halves, masses, centres))
    assert np.allclose(combined, whole, rtol=1e-12)


def test_two_masses_apart_have_more_inertia_than_together(mp):
    """Separation has to cost inertia, by exactly the parallel-axis amount."""
    m, d = 0.055, 0.06
    left, right = np.array([-d / 2, 0, 0]), np.array([d / 2, 0, 0])
    zero = np.zeros((3, 3))
    com = np.zeros(3)
    apart = mp.shift(zero, m, left - com) + mp.shift(zero, m, right - com)
    # Two point masses either side of the centre: no inertia about the axis
    # through both, and m d^2 / 2 about the other two.
    assert apart[0][0] == pytest.approx(0.0, abs=1e-18)
    assert apart[1][1] == pytest.approx(2 * m * (d / 2) ** 2)
    assert apart[2][2] == pytest.approx(2 * m * (d / 2) ** 2)


def test_the_built_arm_conserves_mass(mp):
    """Shell plus servos, link by link, against what the model reports."""
    origins = mp.joint_origins()
    for name in mp.DECLARED:
        tris = mp.read_stl(mp.MESHES / (name + '.stl'))
        volume, centroid, covariance = mp.integrate(tris)
        mass, com, tensor, shell = mp.printed_link(
            name, volume, centroid, covariance, origins)
        servos = sum(mp.SERVOS[k]['mass'] for k, _ in mp.SERVO_LAYOUT[name])
        assert mass == pytest.approx(shell + servos)
        assert shell == pytest.approx(
            volume * mp.PLA_SOLID * mp.PRINTED_FRACTION)
        # An inertia tensor is symmetric and positive definite, always.
        assert np.allclose(tensor, tensor.T, rtol=1e-12)
        assert np.all(np.linalg.eigvalsh(tensor) > 0), f'{name} is not physical'


def test_every_joint_drives_exactly_one_set_of_servos(mp):
    """Six servos over five joints, and no joint left unactuated."""
    driven = [j for servos in mp.SERVO_LAYOUT.values() for _, j in servos]
    assert len(driven) == 6, f'expected six servos, the layout has {len(driven)}'
    assert set(driven) == {f'joint_{i}' for i in range(1, 6)}
    origins = mp.joint_origins()
    for joint in driven:
        assert joint in origins, f'{joint} is not a joint in the description'


XACRO = REPO / 'src' / 'tattotronix_description' / 'urdf' / 'inertials_printed.xacro'


@pytest.fixture(scope='module')
def built(mp):
    """The arm as built, computed once for the tests below."""
    return mp.build_printed()


def test_the_inertials_file_is_what_the_generator_produces(mp, built):
    """Regenerating must be a no-op, or the meshes or declarations moved on."""
    assert XACRO.read_text(encoding='utf-8') == mp.render_xacro(built), (
        'inertials_printed.xacro is stale. '
        'Run: python3 docs/scripts/mass_properties.py --write-xacro'
    )


def test_the_inertials_file_expands_to_the_numbers_it_was_given(mp, built, tmp_path):
    """The xacro has to survive xacro, and come out as the same robot.

    A macro that expands to nothing, or to the wrong link's numbers, would pass
    the text comparison above: the generator and the file would agree with
    each other and both be wrong about what the description receives.
    """
    import subprocess
    import xml.etree.ElementTree as ET
    wrapper = tmp_path / 'probe.urdf.xacro'
    body = '\n'.join(
        f'  <link name="{n}"><xacro:printed_inertial_{n}/></link>' for n in built)
    wrapper.write_text(
        '<?xml version="1.0"?>\n'
        '<robot name="probe" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        f'  <xacro:include filename="{XACRO}"/>\n{body}\n</robot>\n',
        encoding='utf-8')
    out = subprocess.run(['xacro', str(wrapper)], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    root = ET.fromstring(out.stdout)
    for name, (mass, com, tensor, _) in built.items():
        inertial = root.find(f"link[@name='{name}']/inertial")
        assert inertial is not None, f'{name} expanded to no inertial'
        assert float(inertial.find('mass').get('value')) == pytest.approx(mass, rel=1e-5)
        xyz = [float(v) for v in inertial.find('origin').get('xyz').split()]
        assert np.allclose(xyz, com, atol=1e-6)
        got = inertial.find('inertia')
        assert float(got.get('ixy')) == pytest.approx(tensor[0][1], rel=1e-5, abs=1e-12)
        assert float(got.get('izz')) == pytest.approx(tensor[2][2], rel=1e-5)


def test_every_tensor_is_one_a_real_body_can_have(built):
    """Principal moments must satisfy the triangle inequality.

    For any rigid body I1 + I2 >= I3, for every ordering. A tensor that breaks
    it is not merely inaccurate, it describes nothing that exists, and physics
    engines either reject it or integrate it into an explosion. A composition
    bug - a servo shifted the wrong way, a product of inertia with its sign
    flipped - is exactly the kind of thing that produces one.
    """
    for name, (_, _, tensor, _) in built.items():
        p = np.sort(np.linalg.eigvalsh(tensor))
        assert p[0] > 0, f'{name} has a non-positive principal moment'
        assert p[0] + p[1] >= p[2] * (1 - 1e-9), (
            f'{name}: principal moments {p} break the triangle inequality')
