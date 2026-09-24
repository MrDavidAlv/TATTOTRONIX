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
Support for the notebooks: finding the repository, loading the arm, and showing
or exporting what the notebooks draw.

Everything here is plumbing. The robotics lives in the notebooks themselves,
written out so it can be read, and in docs/scripts, which the notebooks import
rather than reimplement: the chain and the dynamic model a notebook builds are
the same objects every study in the repository uses, parsed from the same URDF.

A notebook runs in three places, and this module is what lets one source serve
all of them:

- Google Colab, where the first cell clones the repository and there is no ROS;
- Jupyter on a workstation, inside a checkout;
- tests/test_notebooks.py, which runs each notebook as a plain script, headless,
  so every assert in it certifies the mathematics on every build.

With TATTO_EXPORT=1 in the environment, figures and animations named in show()
and animate() are written to docs/figures, which is how the documentation's
concept figures are produced: by the notebook code itself, not by a copy of it.
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _find_repo():
    here = Path.cwd().resolve()
    for p in [here, *here.parents, Path("/content/TATTOTRONIX")]:
        if (p / "src" / "tattotronix_description").is_dir():
            return p
    raise RuntimeError("run the first cell: it clones the repository in Colab")


REPO = _find_repo()
for sub in ("docs/scripts", "docs/notebooks"):
    if str(REPO / sub) not in sys.path:
        sys.path.insert(0, str(REPO / sub))

URDF = REPO / "docs" / "notebooks" / "tattotronix.urdf"
FIGURES = REPO / "docs" / "figures"
EXPORT = os.environ.get("TATTO_EXPORT") == "1"


def in_notebook():
    """True under Jupyter or Colab, False when run as a script."""
    try:
        from IPython import get_ipython
        return get_ipython() is not None and hasattr(get_ipython(), "kernel")
    except ImportError:
        return False


# ---- the arm -----------------------------------------------------------------

def urdf_text():
    return URDF.read_text(encoding="utf-8")


def arm():
    """The kinematic chain and the dynamic model, from the expanded URDF.

    Built by docs/scripts/kinematics.py and dynamics.py - the same code as
    every study - from the same description robot_state_publisher loads.
    """
    import dynamics
    import kinematics
    text = urdf_text()
    chain = kinematics.Chain(text)
    return chain, dynamics.Model(chain, dynamics.parse_inertials(text))


# ---- geometry ----------------------------------------------------------------

_STL = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def _read_stl(path, scale):
    data = path.read_bytes()
    n = int(np.frombuffer(data, "<u4", 1, 80)[0])
    tris = np.frombuffer(data, _STL, n, 84)["v"].astype(float) * scale
    return _weld(tris)


def _weld(tris, tol=1e-7):
    """Shared vertices and face indices from a triangle soup."""
    pts = tris.reshape(-1, 3)
    key = np.round(pts / tol).astype(np.int64)
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return pts[first], inverse.reshape(-1, 3)


def _cylinder(radius, length, n=24):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[radius * np.cos(a), radius * np.sin(a)]
    v = np.vstack([np.c_[ring, np.full(n, -length / 2)], np.c_[ring, np.full(n, length / 2)],
                   [[0, 0, -length / 2], [0, 0, length / 2]]])
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [[i, j, n + i], [j, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    return v, np.array(f)


def _origin(el):
    from kinematics import homogeneous, rpy_to_matrix
    o = el.find("origin")
    xyz = np.fromstring(o.get("xyz", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
    rpy = np.fromstring(o.get("rpy", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
    return homogeneous(rpy_to_matrix(*rpy), xyz)


def visual_parts(cell=None):
    """Every visual shape of the arm, in its link's frame.

    Returns a list of (link, vertices, faces, material). With cell, in metres,
    each mesh is simplified by clustering its vertices on a grid of that pitch;
    animations use it, because the full meshes are some fifty thousand
    triangles and redrawing them every frame makes a notebook crawl.
    """
    root = ET.fromstring(urdf_text())
    parts = []
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            geo = vis.find("geometry")[0]
            if geo.tag == "mesh":
                rel = geo.get("filename").replace("package://tattotronix_description/", "")
                scale = float(geo.get("scale", "1 1 1").split()[0])
                v, f = _read_stl(REPO / "src" / "tattotronix_description" / rel, scale)
            elif geo.tag == "cylinder":
                v, f = _cylinder(float(geo.get("radius")), float(geo.get("length")))
            else:                                                 # pragma: no cover
                continue
            T = _origin(vis)
            v = v @ T[:3, :3].T + T[:3, 3]
            if cell:
                v, f = decimate(v, f, cell)
            mat = vis.find("material")
            parts.append((link.get("name"), v, f, mat.get("name") if mat is not None else ""))
    return parts


def decimate(v, f, cell):
    """Vertex clustering: merge every vertex in the same grid cell."""
    key = np.floor(v / cell).astype(np.int64)
    _, inverse = np.unique(key, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    counts = np.bincount(inverse)
    nv = np.zeros((counts.size, 3))
    np.add.at(nv, inverse, v)
    nv /= counts[:, None]
    nf = inverse[f]
    keep = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
    nf = np.unique(np.sort(nf[keep], axis=1), axis=0) if keep.any() else nf[keep]
    return nv, nf


def posed(parts, chain, q):
    """The visual parts moved to configuration q, in the world frame."""
    frames = dict(chain.frames(q))
    out = []
    for link, v, f, mat in parts:
        T = frames[link]
        out.append((link, v @ T[:3, :3].T + T[:3, 3], f, mat))
    return out


# ---- showing and exporting ---------------------------------------------------

def show(fig, export=None):
    """Display a matplotlib or plotly figure where it can be seen.

    In a notebook it is displayed. As a script it is closed, and with
    TATTO_EXPORT=1 a matplotlib figure named by export is written to
    docs/figures first.
    """
    if hasattr(fig, "to_plotly_json"):                        # plotly
        if in_notebook():
            fig.show()
        return
    import matplotlib.pyplot as plt
    if EXPORT and export:
        FIGURES.mkdir(exist_ok=True)
        fig.savefig(FIGURES / export, bbox_inches="tight", pad_inches=0.3)
        print("  wrote", export)
    if in_notebook():
        plt.show()
    plt.close(fig)


def animate(anim, export=None, fps=20, dpi=100):
    """Show a matplotlib animation inline, or write it as a GIF when exporting.

    Exported at a modest dpi on purpose: a GIF in a README is downloaded by
    every visitor, and the house 200 dpi makes one several times larger than
    it needs to be.
    """
    import matplotlib.pyplot as plt
    if EXPORT and export:
        FIGURES.mkdir(exist_ok=True)
        anim.save(FIGURES / export, writer="pillow", fps=fps, dpi=dpi)
        print("  wrote", export)
    if in_notebook():
        from IPython.display import HTML, display
        display(HTML(anim.to_jshtml(fps=fps)))
    plt.close(anim._fig)


def display(*objs):
    """IPython's display in a notebook; print otherwise."""
    if in_notebook():
        from IPython.display import display as _display
        _display(*objs)
    else:
        for o in objs:
            print(o)


def table(headers, rows):
    """A small table: rendered markdown in a notebook, aligned text otherwise."""
    rows = [[str(c) for c in r] for r in rows]
    if in_notebook():
        from IPython.display import Markdown
        md = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
        md += ["| " + " | ".join(r) + " |" for r in rows]
        display(Markdown("\n".join(md)))
    else:
        w = [max(len(str(x)) for x in col) for col in zip(headers, *rows)]
        for r in [headers, *rows]:
            print("  ".join(str(c).ljust(n) for c, n in zip(r, w)))


# ---- drawing the arm -----------------------------------------------------------

#: Shades for the arm's materials, from the house palette: the printed shell in
#: a light neutral so frames and traces drawn over it stay readable.
SHADES = {"tattotronix_shell": "#c9c8c3", "tattotronix_steel": "#8a8984",
          "tattotronix_ink": "#2b2b2b"}


#: Light direction for the simple shading below: from above, front and left.
LIGHT = np.array([0.35, -0.55, 0.76])


def shade(tris, colour):
    """One colour per triangle, darker as the face turns from the light.

    matplotlib's 3D polygons are flat-filled, so without this a mesh reads as a
    silhouette. The absolute value makes it independent of winding order, which
    the simplified meshes do not preserve.
    """
    from matplotlib.colors import to_rgb
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    k = 0.52 + 0.48 * np.abs(n @ (LIGHT / np.linalg.norm(LIGHT)))
    return np.clip(np.array(to_rgb(colour))[None, :] * k[:, None], 0, 1)


def draw_arm(ax, parts, alpha=1.0, shaded=True):
    """Add posed visual parts to a matplotlib 3D axis."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    for _, v, f, mat in parts:
        tris = v[f]
        colour = SHADES.get(mat, "#c9c8c3")
        pc = Poly3DCollection(tris, alpha=alpha, linewidths=0,
                              facecolors=shade(tris, colour) if shaded else colour)
        ax.add_collection3d(pc)


def equal_3d(ax, pts, pad=0.02, zoom=1.0):
    """Equal scale on all three axes around a cloud of points, in metres."""
    pts = np.asarray(pts)
    lo, hi = pts.min(axis=0) - pad, pts.max(axis=0) + pad
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=zoom)


def trajectory(name="ros_logo"):
    """A trajectory the draw node ships: joint angles, times, tip path, ink flag."""
    d = np.load(REPO / "src" / "tattotronix_control" / "config" / "trajectories" / f"{name}.npz")
    return {k: d[k] for k in d.files}
