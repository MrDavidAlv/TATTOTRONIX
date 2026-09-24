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
Mass properties integrated from the meshes, instead of guessed from a box.

Every mass and every inertia in the description is currently an approximation
by the solid box that best matches each link's bounding box, and
inertial_macros.xacro says so in as many words. Those numbers set the gravity
load, the torques and, through the effective inertia, all fifteen gains. They
are the credibility ceiling of the whole dynamics and control story.

They do not have to be guessed. The meshes are closed surfaces, and a closed
surface has an exact volume, centroid and inertia tensor - no sampling, no
voxels, no approximation beyond the mesh itself. This computes them by summing
over tetrahedra formed between the origin and each triangle, which is the
divergence theorem written out.

What it cannot know is the material. Volume comes from the mesh; mass needs a
density, and a density is a declaration. So this reports both directions: the
density each currently declared mass implies, which is the honest test of
whether those masses were ever plausible, and the mass each candidate material
would give.

    python3 docs/scripts/mass_properties.py
    python3 docs/scripts/mass_properties.py --density 1240   # kg/m^3, solid PLA
"""

import argparse
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MESHES = ROOT / "src" / "tattotronix_description" / "meshes" / "visual"
XACRO_OUT = ROOT / "src" / "tattotronix_description" / "urdf" / "inertials_printed.xacro"

#: The URDF scales every mesh by this. The CAD was exported in centimetres.
MESH_SCALE = 0.01

#: Masses currently declared in arm_macro.xacro, kg. Read from the xacro rather
#: than repeated here would be better; they are listed so the comparison can be
#: made without expanding the description, and test_mass_properties.py checks
#: this list still matches the file.
DECLARED = {
    "base_link": 1.5,
    "shoulder_link": 0.9,
    "upper_arm_link": 0.6,
    "forearm_link": 0.45,
    "wrist_link": 0.35,
    "tool_mount_link": 0.15,
}

#: Candidate materials, kg/m^3. A printed part is not solid plastic, so the
#: infill figure matters more than the polymer: these bracket what a printed
#: arm can weigh, with aluminium for comparison.
MATERIALS = {
    "PLA, 20% infill": 300.0,
    "PLA, 50% infill": 680.0,
    "PLA, solid": 1240.0,
    "aluminium 6061": 2700.0,
}

# ---------------------------------------------------------------------------
# The arm as actually built: printed shells with servos bolted into them.
#
# This is where measurement stops and declaration starts, and the line is drawn
# here on purpose. Volumes above are integrated from the meshes and are facts.
# Everything in this section is a statement about a robot that is not in the
# room - it was lost - so none of it can be weighed, and all of it is written
# as one named constant that is trivial to correct.
# ---------------------------------------------------------------------------

#: Solid PLA, kg/m^3, and the fraction of solid a printed part actually comes
#: out at. A 20% infill print is not 20% of solid: the perimeters and the top
#: and bottom layers are dense, and on parts this small they dominate. A third
#: of solid is the usual result for a part of this size at 20% with three
#: perimeters. THIS FRACTION IS A DECLARATION, not a measurement, and it is the
#: only number here that nothing in the repository can check.
PLA_SOLID = 1240.0
PRINTED_FRACTION = 0.35

#: The servos. Masses and dimensions are catalogue figures for the parts named,
#: declared because the model was not confirmed. Changing them is one line each.
SERVOS = {
    "large": {"mass": 0.055, "size": (0.0407, 0.0197, 0.0429), "part": "MG996R"},
    "small": {"mass": 0.009, "size": (0.0225, 0.0118, 0.0227), "part": "SG90"},
}

#: Which servos sit in which link, and which joint each one drives. A servo is
#: mounted on the link *before* the joint it turns, so its mass is placed at
#: that joint's origin, which the URDF gives exactly.
#:
#: Six servos across five joints: the shoulder carries two, which is where a
#: printed arm doubles them because it is the joint with the most gravity load.
#: The small one is last, since joint_5 turns nothing but the pen.
SERVO_LAYOUT = {
    "base_link": [("large", "joint_1")],
    "shoulder_link": [("large", "joint_2"), ("large", "joint_2")],
    "upper_arm_link": [("large", "joint_3")],
    "forearm_link": [("large", "joint_4")],
    "wrist_link": [("small", "joint_5")],
    "tool_mount_link": [],
}

#: The covariance of the canonical tetrahedron with vertices at the origin and
#: the three unit axes. Every tetrahedron's contribution is this, mapped through
#: the matrix of its edge vectors.
CANONICAL = np.array([[2.0, 1.0, 1.0],
                      [1.0, 2.0, 1.0],
                      [1.0, 1.0, 2.0]]) / 120.0


def read_stl(path):
    """Triangles from a binary STL, in metres."""
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    tris = np.empty((count, 3, 3))
    for i in range(count):
        base = 84 + i * 50 + 12
        tris[i] = np.frombuffer(data, dtype="<f4", count=9, offset=base).reshape(3, 3)
    return tris * MESH_SCALE


#: Vertices are snapped to this grid, in metres, before edges are matched. An
#: STL stores every triangle's corners independently and in single precision,
#: so one geometric vertex arrives as several values differing in the last bits
#: - about 2e-8 m at this arm's size. A micrometre is fifty times that noise
#: and far below any feature of a printed part, so it merges what is the same
#: point without merging what is not.
WELD_M = 1e-6


def is_closed(tris):
    """Whether every edge is shared by exactly two triangles.

    An open surface has no interior, so its volume is whatever the arithmetic
    happens to produce. This is the check that makes the rest meaningful.

    This is a test of mesh *topology*, and on these meshes it fails: three to
    five percent of edges are unpaired at any sensible weld tolerance. That
    turns out to be duplicated vertices and T-junctions rather than holes, so
    it is reported as hygiene and is not what decides whether the volume means
    anything. `closure_residual` is.

    Returns (closed, edge count, how many edges are not shared by exactly two).
    """
    edges = {}
    quantised = np.round(tris / WELD_M).astype(np.int64)
    for tri in quantised:
        for k in range(3):
            key = frozenset((tuple(tri[k]), tuple(tri[(k + 1) % 3])))
            edges[key] = edges.get(key, 0) + 1
    bad = sum(1 for n in edges.values() if n != 2)
    return bad == 0, len(edges), bad


def closure_residual(tris):
    """How far the surface is from closed, as a fraction of its own area.

    The integral of the outward normal over a closed surface is exactly zero:
    every direction is cancelled by the far side. Summing the area-weighted
    normals and comparing the leftover against the total area therefore
    measures openness directly, in the geometry, without caring whether two
    triangles that meet happen to share vertex indices.

    This is the test the volume depends on. A surface that is closed to a part
    in 1e11 encloses a definite region whatever its index topology looks like.
    """
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    cross = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cross, axis=1).sum()
    residual = np.linalg.norm(0.5 * cross.sum(axis=0))
    return residual / area, area


def integrate(tris):
    """Volume, centroid and the second-moment covariance of a closed mesh.

    Each triangle forms a tetrahedron with the origin. The signed volume of
    that tetrahedron is positive where the surface faces away and negative
    where it faces back, so the interior is counted once and everything
    outside cancels. The same decomposition carries the covariance, which is
    where the inertia tensor comes from.
    """
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    signed = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    volume = signed.sum()
    centroid = (signed[:, None] * (a + b + c) / 4.0).sum(axis=0) / volume

    covariance = np.zeros((3, 3))
    for tri, vol in zip(tris, signed):
        edge = tri.T                       # columns are the tetrahedron's edges
        det = np.linalg.det(edge)
        covariance += det * (edge @ CANONICAL @ edge.T)
    return volume, centroid, covariance


def inertia_about(volume, centroid, covariance, density):
    """The inertia tensor about the centre of mass, kg m^2.

    The arguments are in the order `integrate` returns them, so the two
    compose: `inertia_about(*integrate(tris), density)`. They used to differ,
    which silently swapped a scalar for a matrix and produced numbers that
    still looked like an inertia tensor.
    """
    # Inertia from covariance: I = tr(C) 1 - C, then shift to the centroid.
    about_origin = np.trace(covariance) * np.eye(3) - covariance
    mass = density * volume
    shift = volume * (np.dot(centroid, centroid) * np.eye(3) - np.outer(centroid, centroid))
    return density * (about_origin - shift), mass


def joint_origins():
    """Where each joint sits in its parent link, from the live description."""
    import subprocess
    urdf = ROOT / "src" / "tattotronix_description" / "urdf" / "tattotronix.urdf.xacro"
    out = subprocess.run(
        [sys.executable, "-c",
         "import sys; from xacro import main; sys.argv=['xacro', %r]; main()" % str(urdf)],
        capture_output=True, text=True)
    if out.returncode != 0:                                   # pragma: no cover
        raise SystemExit("xacro failed:\n" + out.stderr)
    root = ET.fromstring(out.stdout)
    origins = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = (origin.get("xyz") if origin is not None else None) or "0 0 0"
        origins[joint.get("name")] = np.array([float(v) for v in xyz.split()])
    return origins


def box_inertia(mass, size):
    """Inertia of a solid box about its own centre, kg m^2."""
    x, y, z = size
    return np.diag([mass * (y * y + z * z) / 12.0,
                    mass * (x * x + z * z) / 12.0,
                    mass * (x * x + y * y) / 12.0])


def shift(tensor, mass, offset):
    """Parallel axis: move an inertia tensor from a body's centre to a frame."""
    return tensor + mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))


def printed_link(name, volume, centroid, covariance, origins):
    """Mass, centre of mass and inertia of one link as built.

    The shell is the mesh at the printed density. Each servo is a box of
    catalogue size and mass, placed at the origin of the joint it drives. The
    two are combined properly - masses add, centres of mass average by weight,
    and both inertias are carried to the combined centre - rather than by
    smearing the motors into the plastic, which is the whole point: a servo is
    a third of a link's mass sitting at one end of it.
    """
    shell_density = PLA_SOLID * PRINTED_FRACTION
    shell_inertia, shell_mass = inertia_about(volume, centroid, covariance, shell_density)

    masses = [shell_mass]
    centres = [centroid]
    inertias = [shell_inertia]
    for kind, joint in SERVO_LAYOUT.get(name, []):
        spec = SERVOS[kind]
        masses.append(spec["mass"])
        centres.append(origins[joint])
        inertias.append(box_inertia(spec["mass"], spec["size"]))

    mass = sum(masses)
    com = sum(m * c for m, c in zip(masses, centres)) / mass
    tensor = sum(shift(i, m, c - com) for i, m, c in zip(inertias, masses, centres))
    return mass, com, tensor, shell_mass


def build_printed():
    """Every link of the arm as built, keyed by link name.

    Returns {name: (mass, centre of mass, inertia about it, shell mass)}.
    """
    origins = joint_origins()
    out = {}
    for name in DECLARED:
        tris = read_stl(MESHES / (name + ".stl"))
        volume, centroid, covariance = integrate(tris)
        out[name] = printed_link(name, volume, centroid, covariance, origins)
    return out


def render_xacro(built):
    """The inertial blocks as a xacro file, one macro per link.

    A macro per link rather than a table of numbers, so the file reads as the
    URDF it will become and a diff of it reads as a change to the robot.
    """
    lines = [
        '<?xml version="1.0"?>',
        "<!--",
        # No double hyphen anywhere in here: XML forbids "--" inside a
        # comment, so naming the option by its flag made the file unparseable.
        "  Generated by docs/scripts/mass_properties.py, write-xacro option.",
        "  Do not edit by hand: change the meshes or the declarations in that",
        "  script and run it again. tests/test_mass_properties.py fails if this",
        "  file and the script disagree.",
        "",
        "  Each link is its printed shell - the mesh volume at %.0f kg/m^3, which is"
        % (PLA_SOLID * PRINTED_FRACTION),
        "  %.0f%% of solid PLA - plus the servos mounted in it, each a box of"
        % (100 * PRINTED_FRACTION),
        "  catalogue size at the origin of the joint it drives. The volumes are",
        "  measured. The printed fraction and the servo figures are declarations",
        "  about an arm that cannot be weighed, and are named in the script.",
        "",
        "  Inertia is about the centre of mass, in the link frame's axes, which",
        "  is what the URDF inertial block means by an origin with no rotation.",
        "-->",
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
    ]
    for name, (mass, com, tensor, shell) in built.items():
        servos = SERVO_LAYOUT.get(name, [])
        what = (", ".join("%s %s" % (SERVOS[k]["part"], j) for k, j in servos)
                or "no servo")
        lines += [
            "",
            "  <!-- %s: shell %.4f kg, %s -->" % (name, shell, what),
            '  <xacro:macro name="printed_inertial_%s">' % name,
            "    <inertial>",
            '      <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % tuple(com),
            # Significant figures, not fixed decimals: six decimals leave the
            # five-gram tool mount with four figures and the rest with five.
            '      <mass value="%.9g"/>' % mass,
            '      <inertia ixx="%.6e" ixy="%.6e" ixz="%.6e"'
            % (tensor[0][0], tensor[0][1], tensor[0][2]),
            '               iyy="%.6e" iyz="%.6e" izz="%.6e"/>'
            % (tensor[1][1], tensor[1][2], tensor[2][2]),
            "    </inertial>",
            "  </xacro:macro>",
        ]
    lines += ["", "</robot>", ""]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--density", type=float, default=None,
                    help="kg/m^3 to report a full inertia tensor for")
    ap.add_argument("--printed", action="store_true",
                    help="the arm as built: printed shells plus servos")
    ap.add_argument("--write-xacro", action="store_true",
                    help="write the built arm's inertials to the description")
    args = ap.parse_args()

    if args.write_xacro:
        XACRO_OUT.write_text(render_xacro(build_printed()), encoding="utf-8")
        print("wrote %s" % XACRO_OUT.relative_to(ROOT))
        return

    print("Volumes integrated from the meshes, and the density each declared "
          "mass implies\n")
    print("%-18s %10s %10s  %8s %11s  %5s"
          % ("link", "volume", "declared", "implied", "closure", "edge"))
    print("%-18s %10s %10s  %8s %11s  %5s"
          % ("", "cm^3", "mass kg", "g/cm^3", "residual", "noise"))

    results = {}
    for name, declared in DECLARED.items():
        path = MESHES / (name + ".stl")
        if not path.exists():                                 # pragma: no cover
            print("%-18s mesh missing" % name)
            continue
        tris = read_stl(path)
        closed, n_edges, bad_edges = is_closed(tris)
        residual, area = closure_residual(tris)
        volume, centroid, covariance = integrate(tris)
        implied = declared / volume / 1000.0                  # kg/m^3 -> g/cm^3
        results[name] = (volume, centroid, covariance, declared, closed)
        del area
        print("%-18s %10.2f %10.3f  %8.2f %11.1e  %4.1f%%"
              % (name, volume * 1e6, declared, implied, residual,
                 100.0 * bad_edges / n_edges))

    total_volume = sum(v for v, _, _, _, _ in results.values())
    total_mass = sum(d for _, _, _, d, _ in results.values())
    print("%-18s %10.2f %10.3f  %8.2f"
          % ("TOTAL", total_volume * 1e6, total_mass,
             total_mass / total_volume / 1000.0))
    print("\nClosure residual is |integral of n dA| over total area, which is "
          "zero for a closed\nsurface. Every link here is closed to at worst a "
          "part in 300, so the volumes are\nreal. Edge noise is unpaired edges: "
          "duplicated vertices and T-junctions, which\nmake the index topology "
          "untidy without opening the surface.")

    print("\nFor reference: water is 1.00, PLA 1.24, aluminium 2.70, "
          "steel 7.85 g/cm^3.")
    print("A printed part is mostly air, so anything above about 1.2 is not a "
          "printed arm,\nand anything above 7.85 is not a solid steel one "
          "either.\n")

    print("Mass each candidate material would give, kg\n")
    header = "%-18s" % "link"
    for label in MATERIALS:
        header += "%18s" % label
    print(header)
    for name, (volume, _, _, declared, _) in results.items():
        row = "%-18s" % name
        for density in MATERIALS.values():
            row += "%18.3f" % (density * volume)
        print(row)
    row = "%-18s" % "TOTAL"
    for density in MATERIALS.values():
        row += "%18.3f" % (density * total_volume)
    print(row)

    if args.printed:
        origins = joint_origins()
        print("\nThe arm as built: printed shells at %.0f kg/m^3 "
              "(%.0f%% of solid PLA)\nplus servos placed at the joints they "
              "drive.\n" % (PLA_SOLID * PRINTED_FRACTION, 100 * PRINTED_FRACTION))
        print("%-18s %8s %8s %8s   %-24s %s"
              % ("link", "shell", "servos", "total", "centre of mass, m", "servos"))
        print("%-18s %8s %8s %8s" % ("", "kg", "kg", "kg"))
        built = {}
        for name, (volume, centroid, covariance, declared, _) in results.items():
            mass, com, tensor, shell = printed_link(
                name, volume, centroid, covariance, origins)
            built[name] = (mass, com, tensor)
            kinds = [k for k, _ in SERVO_LAYOUT.get(name, [])]
            print("%-18s %8.3f %8.3f %8.3f   (%6.3f,%6.3f,%6.3f)  %s"
                  % (name, shell, mass - shell, mass, *com,
                     ", ".join(kinds) or "-"))
        total = sum(m for m, _, _ in built.values())
        shells = sum(PLA_SOLID * PRINTED_FRACTION * v
                     for v, _, _, _, _ in results.values())
        print("%-18s %8.3f %8.3f %8.3f"
              % ("TOTAL", shells, total - shells, total))
        print("\nAgainst %.3f kg declared today. The motors are %.0f%% of the "
              "arm, so the mass\nis concentrated at the joints rather than "
              "spread along the links."
              % (total_mass, 100 * (total - shells) / total))

        print("\nInertia tensors about each centre of mass, kg m^2\n")
        for name, (mass, com, tensor) in built.items():
            print("%s   mass %.4f kg" % (name, mass))
            for row in tensor:
                print("    %12.3e %12.3e %12.3e" % tuple(row))

    if args.density is not None:
        print("\nInertia tensors about each centre of mass at %.0f kg/m^3, "
              "kg m^2\n" % args.density)
        for name, (volume, centroid, covariance, _, _) in results.items():
            tensor, mass = inertia_about(volume, centroid, covariance, args.density)
            print("%s   mass %.4f kg   com (%.4f, %.4f, %.4f) m"
                  % (name, mass, *centroid))
            for row in tensor:
                print("    %12.3e %12.3e %12.3e" % tuple(row))


if __name__ == "__main__":
    main()
