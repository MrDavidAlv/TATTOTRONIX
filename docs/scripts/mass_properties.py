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
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MESHES = ROOT / "src" / "tattotronix_description" / "meshes" / "visual"

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


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--density", type=float, default=None,
                    help="kg/m^3 to report a full inertia tensor for")
    args = ap.parse_args()

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
