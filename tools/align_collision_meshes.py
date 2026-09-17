#!/usr/bin/env python3
"""
Bring every collision DAE into the frame of the visual STL it belongs to.

Why this exists
---------------
The CAD exports that seeded this project were written out twice, once as STL
for the visual geometry and once as COLLADA for the collision geometry, and the
two runs did not agree on axes. Every DAE declares Z_UP while its coordinates
are Y-up, so the collision shape of most links is rotated 90 degrees about X
relative to what is drawn; the wrist is rotated about Y instead; a few carry a
translation on top. Nothing complains, because a URDF never checks that a
link's two geometries describe the same object. The cost shows up in
simulation, where the arm collides with a shape that is not the arm.

The meshes themselves are sound: each DAE has exactly the same triangle count
and the same surface area as its STL, so the two differ by a rigid transform
and nothing else. This script finds that transform and bakes it into the DAE.

How it works
------------
For each link, the 48 signed axis permutations are tried. A candidate has to
reproduce the STL bounding box, and then it has to place the DAE triangle
centroids on top of the STL ones, checked through a spatial hash at 0.2 mm.
Only a transform that matches every sampled triangle is applied, so a mesh that
is not simply misoriented is reported and left alone rather than mangled.

Run from the repository root. It is idempotent: a DAE already in its STL frame
resolves to the identity and is skipped.
"""

import argparse
import glob
import itertools
import os
import struct
import xml.etree.ElementTree as ET

COLLADA_NS = 'http://www.collada.org/2005/11/COLLADASchema'
VISUAL_DIR = 'src/tattotronix_description/meshes/visual'
COLLISION_DIR = 'src/tattotronix_description/meshes/collision'

# Triangle centroids have to land this close, in centimetres, for the transform
# to count as a match. The DAEs store fewer significant digits than the STLs, so
# an exact comparison would reject a correct transform.
TOLERANCE_CM = 0.02


def read_stl(path):
    with open(path, 'rb') as handle:
        header = handle.read(84)
        count = struct.unpack('<I', header[80:84])[0]
        triangles = []
        for _ in range(count):
            chunk = handle.read(50)
            if len(chunk) < 50:
                break
            values = struct.unpack('<12fH', chunk)
            triangles.append([list(values[3:6]), list(values[6:9]), list(values[9:12])])
    return triangles


def read_dae(path):
    ET.register_namespace('', COLLADA_NS)
    tree = ET.parse(path)
    root = tree.getroot()

    positions = normals = None
    for source in root.iter('{%s}source' % COLLADA_NS):
        array = source.find('{%s}float_array' % COLLADA_NS)
        accessor = source.find('.//{%s}accessor' % COLLADA_NS)
        if array is None or accessor is None:
            continue
        params = [p.get('name') for p in accessor.findall('{%s}param' % COLLADA_NS)]
        if params[:3] != ['X', 'Y', 'Z']:
            continue
        if 'normal' in (source.get('id') or ''):
            normals = array
        else:
            positions = array

    triangles_node = root.find('.//{%s}triangles' % COLLADA_NS)
    inputs = triangles_node.findall('{%s}input' % COLLADA_NS)
    stride = max(int(i.get('offset')) for i in inputs) + 1
    vertex_offset = [int(i.get('offset')) for i in inputs
                     if i.get('semantic') == 'VERTEX'][0]
    indices = [int(x) for x in triangles_node.find('{%s}p' % COLLADA_NS).text.split()]

    flat = [float(x) for x in positions.text.split()]
    points = [flat[i:i + 3] for i in range(0, len(flat), 3)]
    count = int(triangles_node.get('count'))
    triangles = [[points[indices[t * 3 * stride + k * stride + vertex_offset]]
                  for k in range(3)] for t in range(count)]
    return tree, positions, normals, triangles


def signed_permutations():
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            matrix = [[0] * 3 for _ in range(3)]
            for row in range(3):
                matrix[row][permutation[row]] = signs[row]
            yield matrix


def determinant(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def rotate(matrix, vector):
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def bounds(points):
    low = [min(p[i] for p in points) for i in range(3)]
    high = [max(p[i] for p in points) for i in range(3)]
    return low, high


def centroid(triangle):
    return [sum(v[i] for v in triangle) / 3.0 for i in range(3)]


def describe(matrix):
    axes = 'xyz'
    parts = []
    for row in range(3):
        column = next(c for c in range(3) if matrix[row][c] != 0)
        parts.append(('-' if matrix[row][column] < 0 else '+') + axes[column])
    return '(%s, %s, %s)' % tuple(parts)


def build_index(triangles):
    index = {}
    for triangle in triangles:
        point = centroid(triangle)
        key = tuple(int(round(c * 10)) for c in point)
        index.setdefault(key, []).append(point)
    return index


def matches(index, point):
    key = tuple(int(round(c * 10)) for c in point)
    for offset in itertools.product((-1, 0, 1), repeat=3):
        neighbour = tuple(key[i] + offset[i] for i in range(3))
        for candidate in index.get(neighbour, ()):
            if max(abs(candidate[i] - point[i]) for i in range(3)) < TOLERANCE_CM:
                return True
    return False


def solve(stl_triangles, dae_triangles):
    """Return (matrix, translation) taking the DAE into the STL frame, or None."""
    stl_points = [v for t in stl_triangles for v in t]
    dae_points = [v for t in dae_triangles for v in t]
    stl_low, stl_high = bounds(stl_points)
    stl_size = [stl_high[i] - stl_low[i] for i in range(3)]
    index = build_index(stl_triangles)
    sample = dae_triangles[::7] or dae_triangles

    for matrix in signed_permutations():
        if determinant(matrix) != 1:
            continue
        low, high = bounds([rotate(matrix, p) for p in dae_points])
        if any(abs((high[i] - low[i]) - stl_size[i]) > 0.05 for i in range(3)):
            continue
        translation = [stl_low[i] - low[i] for i in range(3)]
        if all(matches(index, [rotate(matrix, v)[i] + translation[i] for i in range(3)])
               for v in map(centroid, sample)):
            return matrix, translation
    return None


def transform_array(array, matrix, translation):
    values = [float(x) for x in array.text.split()]
    out = []
    for i in range(0, len(values), 3):
        rotated = rotate(matrix, values[i:i + 3])
        out.extend(rotated[j] + translation[j] for j in range(3))
    array.text = ' '.join('%.6f' % v for v in out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true',
                        help='rewrite the DAE files instead of only reporting')
    args = parser.parse_args()

    for stl_path in sorted(glob.glob(os.path.join(VISUAL_DIR, '*.stl'))):
        name = os.path.basename(stl_path)[:-4]
        dae_path = os.path.join(COLLISION_DIR, name + '.dae')
        if not os.path.exists(dae_path):
            print('%-22s no collision mesh' % name)
            continue

        stl_triangles = read_stl(stl_path)
        tree, positions, normals, dae_triangles = read_dae(dae_path)

        if len(stl_triangles) != len(dae_triangles):
            print('%-22s triangle counts differ (%d vs %d), left alone'
                  % (name, len(stl_triangles), len(dae_triangles)))
            continue

        solution = solve(stl_triangles, dae_triangles)
        if solution is None:
            print('%-22s no rigid transform found, left alone' % name)
            continue

        matrix, translation = solution
        identity = describe(matrix) == '(+x, +y, +z)' and all(
            abs(t) < 1e-4 for t in translation)
        if identity:
            print('%-22s already aligned' % name)
            continue

        print('%-22s %s + (%.3f, %.3f, %.3f) cm%s'
              % (name, describe(matrix), *translation,
                 '' if args.apply else '   [dry run]'))
        if args.apply:
            transform_array(positions, matrix, translation)
            if normals is not None:
                transform_array(normals, matrix, [0.0, 0.0, 0.0])
            tree.write(dae_path, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
