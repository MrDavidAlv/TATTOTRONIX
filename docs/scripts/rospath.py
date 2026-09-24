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

"""Artwork to tattoo toolpath.

The pipeline is mask driven on purpose. Anything that can be turned into a
binary ink mask - a PNG, a rasterised SVG, or the parametric stand-in below -
goes through the same fill, ordering and lead-in code, so swapping the official
ROS artwork in later changes one function and nothing else.

    mask -> connected components -> boundary pass + scanline fill -> ordered
    path with retract / travel / plunge moves

A tattoo machine does not draw outlines, it packs. The fill is therefore a
boustrophedon at the needle's stroke pitch, preceded by one pass around the
boundary, which is how a rotary machine lines and then shades a solid.

Artwork: `ros_logo_mask()` is the official mark, rasterised from the SVG that
`fetch_artwork.sh` downloads from ros-infrastructure/artwork. `placeholder_mask()`
is the 3x3 dot lattice that stood in for it while the chain was being built; it
is kept because it is a useful small case, and it is labelled a stand-in
wherever it appears. Nothing downstream knows which one it is drawing, which is
the point.

Units are millimetres in the panel plane; the caller places that plane in the
world.
"""

from pathlib import Path

import numpy as np
from scipy import ndimage

ARTWORK = Path(__file__).resolve().parents[1] / "artwork"

# --- process parameters ------------------------------------------------------

# Spacing between adjacent fill passes. Note this is not coverage: a 0.3 mm
# needle at 1.2 mm leaves 0.9 mm of skin between passes, which is hatching. A
# pitch at or below the line width fills solid, at four times the marking time.
STROKE_PITCH = 1.2

# Arc length the path is resampled to. Swept in resample_study.py: going from
# 0.6 to 0.15 mm takes the worst marking error from 179 to 19 um and drops peak
# torque 22%, for 3.7x the points and 4% of cycle time. Almost none of that is
# geometry - the win is that the velocity feedforward comes from finite
# differences on this path, and at 0.6 mm the reference velocity is a staircase.
POINT_STEP = 0.15

CLEARANCE = 8.0         # travel height above the surface, mm
PLUNGE_DEPTH = 1.5      # how far below the surface the needle is driven, mm

# Last part of the plunge, taken at marking feed rather than travel feed. Swept
# in approach_study.py: 4 mm is where the worst needle entry falls under the
# 0.3 mm line width, for 14% of cycle time.
APPROACH = 4.0

MIN_BLOB_MM2 = 4.0      # ignore specks smaller than this

# --- placeholder artwork -----------------------------------------------------

DOT_PITCH = 22.0
COL_PITCH = 22.0
COL_SHEAR = -11.0
DOT_RADIUS = 6.0


def placeholder_mask(res=0.25):
    """A sheared 3x3 dot lattice. A stand-in, not the ROS logo."""
    centres = []
    for col in range(3):
        for row in range(3):
            centres.append(((col - 1) * COL_PITCH,
                            (row - 1) * DOT_PITCH + (col - 1) * COL_SHEAR))
    c = np.array(centres, float)
    c -= c.mean(axis=0)
    lo = c.min(axis=0) - DOT_RADIUS - 2
    hi = c.max(axis=0) + DOT_RADIUS + 2
    xs = np.arange(lo[0], hi[0], res)
    ys = np.arange(lo[1], hi[1], res)[::-1]   # rows run downwards, as in an image
    X, Y = np.meshgrid(xs, ys)
    mask = np.zeros(X.shape, bool)
    for cx, cy in c:
        mask |= ((X - cx) ** 2 + (Y - cy) ** 2) <= DOT_RADIUS ** 2
    return mask, (xs, ys)


def _otsu(g):
    """Threshold that best separates the grey histogram into two classes."""
    hist, _ = np.histogram(g, bins=256, range=(0, 256))
    total = hist.sum()
    w = np.cumsum(hist)
    m = np.cumsum(hist * np.arange(256))
    with np.errstate(invalid="ignore", divide="ignore"):
        between = (m[-1] * w / total - m) ** 2 / (w * (total - w))
    return int(np.nanargmax(between))


def _grey(path, width_mm, res):
    """Greyscale image resampled onto the panel grid, in millimetres."""
    from PIL import Image
    im = Image.open(path)
    if im.mode in ("RGBA", "LA", "P"):
        im = im.convert("RGBA")
        bg = Image.new("RGBA", im.size, (255, 255, 255, 255))
        im = Image.alpha_composite(bg, im)
    g = np.asarray(im.convert("L"), float)
    h_px, w_px = g.shape
    out_w = int(round(width_mm / res))
    out_h = int(round(h_px * (width_mm / w_px) / res))
    G = ndimage.zoom(g, (out_h / h_px, out_w / w_px), order=1)
    xs = (np.arange(out_w) - out_w / 2) * res
    ys = (np.arange(out_h) - out_h / 2)[::-1] * res   # image rows run downwards
    return G, (xs, ys)


def _mask_threshold(G, threshold=None):
    """Solid regions, split at `threshold` or at Otsu's.

    The polarity is decided rather than assumed: **ink is the minority**. A
    drawing has less ink on it than ground, so whichever side of the threshold
    covers less than half the image is the ink. Without that, a logo on a black
    field comes out as a filled rectangle with the artwork punched out of it.
    """
    t = _otsu(G) if threshold is None else threshold
    dark = G < t
    return dark if dark.mean() <= 0.5 else ~dark


def _mask_edges(G, sigma=2.0, k=1.0):
    """Line art: gradient magnitude above k standard deviations.

    For a photograph this is the honest tool. Thresholding a continuous-tone
    image fuses hair and dark clothing into one blob, and a local threshold
    recovers the detail at a cost nobody will pay - on a 150 mm portrait it came
    to 34 hours of marking. Edges give what a tattooist would actually draw from
    a photo, and in a fraction of the time.
    """
    smooth = ndimage.gaussian_filter(G, sigma)
    mag = np.hypot(ndimage.sobel(smooth, 0), ndimage.sobel(smooth, 1))
    return mag > (mag.mean() + k * mag.std())


def from_image(path, width_mm, res=0.25, method="threshold", threshold=None):
    """Binary ink mask from any image, scaled to `width_mm` across.

    `method` is "threshold" for artwork with solid areas - logos, lettering,
    flat illustration - or "edges" for photographs. Passing `threshold`
    overrides Otsu's for the threshold method.

    Takes a path or an open file object. Anything with an alpha channel is
    composited onto white first, so a transparent PNG behaves.
    """
    G, grid = _grey(path, width_mm, res)
    if method == "threshold":
        return _mask_threshold(G, threshold), grid
    if method == "edges":
        return _mask_edges(G), grid
    raise ValueError(f"unknown method {method!r}; use 'threshold' or 'edges'")


def from_svg(path, width_mm, res=0.25, threshold=128):
    """Binary ink mask from an SVG, rasterised at twice the sampling pitch.

    Rasterising at the mask resolution would alias every curve the vector file
    describes exactly; at 2x and then downsampling, an edge pixel is decided by
    four subpixels rather than one, which is what keeps the counters of the R
    and the O from closing up at small sizes.
    """
    try:
        import cairosvg
    except ImportError as e:                                  # pragma: no cover
        raise RuntimeError(
            "rasterising an SVG needs cairosvg: pip install --user cairosvg"
        ) from e
    import io
    px = int(round(2 * width_mm / res))
    buf = cairosvg.svg2png(url=str(path), output_width=px)
    return from_image(io.BytesIO(buf), width_mm, res=res, threshold=threshold)


def ros_logo_mask(width_mm=150.0, res=0.25):
    """The official ROS logo, as an ink mask on the panel.

    Not vendored: the mark is CC BY-NC 4.0 and covered by the ROS trademark
    policy, and this repository is Apache-2.0. `fetch_artwork.sh` downloads it.
    """
    svg = ARTWORK / "ros_logo.svg"
    if not svg.exists():
        raise FileNotFoundError(
            f"{svg} is missing. Run docs/scripts/fetch_artwork.sh to "
            "download the official mark from ros-infrastructure/artwork."
        )
    return from_svg(svg, width_mm, res=res)


# --- fill --------------------------------------------------------------------

def _resample(points, step=None):
    """Constant arc length resampling of a polyline.

    `step` defaults to POINT_STEP, read here rather than in the signature.
    Python evaluates a default argument once, when the function is defined, so
    `step=POINT_STEP` would freeze whatever the constant was at import and
    silently ignore anyone who set it afterwards - which is exactly what a
    sweep over the step does.
    """
    step = POINT_STEP if step is None else step
    if len(points) < 2:
        return points
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < step:
        return points[[0, -1]]
    t = np.arange(0.0, s[-1], step)
    return np.column_stack([np.interp(t, s, points[:, i]) for i in range(points.shape[1])])


# Moore neighbourhood, clockwise from east. Rows run downwards, as in an image,
# so east -> south-east -> south is clockwise on screen.
_MOORE = ((0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1))
_MOORE_INDEX = {d: i for i, d in enumerate(_MOORE)}


def _moore_trace(padded, start, backtrack):
    """One closed boundary loop through `start`, as a list of (row, col).

    Moore neighbour tracing: stand on a boundary pixel, sweep the eight
    neighbours clockwise starting from wherever we came in, and step onto the
    first filled one. Jacob's stopping criterion ends the walk when the start
    pixel is re-entered *from the same neighbour*; stopping merely on reaching
    the start again cuts loops short on a one pixel wide neck, which is exactly
    what serifs and thin strokes are made of.

    `backtrack` is the empty pixel we are looking from, and it is what decides
    which side of the boundary the walk hugs: outside it for an outer loop,
    inside it for a hole.
    """
    loop = [start]
    s, b = start, backtrack
    entry = (start, backtrack)
    guard = 4 * padded.size
    for _ in range(guard):
        k = _MOORE_INDEX[(b[0] - s[0], b[1] - s[1])]
        prev = b
        for j in range(1, 9):
            off = _MOORE[(k + j) % 8]
            cand = (s[0] + off[0], s[1] + off[1])
            if padded[cand]:
                break
            prev = cand
        else:
            return loop                      # isolated pixel, nothing to trace
        if (cand, prev) == entry:
            return loop
        s, b = cand, prev
        loop.append(s)
    return loop                              # guard tripped; caller still gets a path


def _loops(blob):
    """Outer boundary and one loop per hole, in (row, col) pixel coordinates.

    Foreground is traced 8-connected and holes are found 4-connected, which is
    the pairing that keeps a diagonal pixel bridge from reading as solid on one
    side and as a hole on the other.
    """
    pad = np.pad(blob, 1)
    out = []

    r, c = np.nonzero(pad)
    first = np.argmin(r * pad.shape[1] + c)  # topmost, then leftmost
    s = (int(r[first]), int(c[first]))
    out.append(_moore_trace(pad, s, (s[0], s[1] - 1)))   # its left is empty by construction

    holes, n = ndimage.label(~pad)           # 4-connected: default structure
    outside = holes[0, 0]
    for k in range(1, n + 1):
        if k == outside:
            continue
        hr, hc = np.nonzero(holes == k)
        h = np.argmin(hr * pad.shape[1] + hc)
        hs = (int(hr[h]), int(hc[h]))
        # The pixel above the topmost hole pixel is filled: were it empty it
        # would be 4-connected to the hole and part of it.
        out.append(_moore_trace(pad, (hs[0] - 1, hs[1]), hs))

    return [[(rr - 1, cc - 1) for rr, cc in loop] for loop in out]


def _smooth_closed(P, passes=2):
    """Take the pixel staircase off a closed polyline.

    A [1 2 1]/4 pass around the loop. Each pass moves a point by at most half
    the grid step, so at 0.25 mm the whole correction stays under the 0.3 mm
    line width; it removes the sawtooth without walking the boundary inwards
    the way an erosion would.
    """
    if len(P) < 5:
        return P
    for _ in range(passes):
        P = 0.25 * np.roll(P, 1, axis=0) + 0.5 * P + 0.25 * np.roll(P, -1, axis=0)
    return P


def _boundary_loops(blob, xs, ys):
    """Closed passes around a region: its outline first, then any holes.

    Traced, not sorted by angle about the centroid. The old ordering happened
    to work on the placeholder's dots because a disc is convex, and produced a
    star shaped scribble on anything that is not - which is every letterform.
    """
    segs = []
    for loop in _loops(blob):
        if len(loop) < 8:
            continue
        rr = np.fromiter((p[0] for p in loop), int, len(loop))
        cc = np.fromiter((p[1] for p in loop), int, len(loop))
        P = np.column_stack([xs[cc], ys[rr]])
        P = _smooth_closed(P)
        segs.append(_resample(np.vstack([P, P[:1]])))
    return segs


def _scanlines(blob, xs, ys, pitch=None):
    """Boustrophedon fill of one region at the stroke pitch.

    `pitch` defaults to STROKE_PITCH, read at call time for the same reason
    `_resample` reads POINT_STEP at call time.
    """
    pitch = STROKE_PITCH if pitch is None else pitch
    res = float(ys[0] - ys[1]) if len(ys) > 1 else 1.0
    every = max(int(round(pitch / abs(res))), 1)
    runs = []
    for i in range(0, blob.shape[0], every):
        row = blob[i]
        if not row.any():
            continue
        idx = np.nonzero(row)[0]
        splits = np.split(idx, np.nonzero(np.diff(idx) > 1)[0] + 1)
        segs = [(xs[s[0]], xs[s[-1]], ys[i]) for s in splits if len(s) > 1]
        runs.append(segs)
    path = []
    flip = False
    for segs in runs:
        segs = sorted(segs, key=lambda s: s[0], reverse=flip)
        for x0, x1, y in segs:
            a, b = (x1, x0) if flip else (x0, x1)
            path.append(_resample(np.array([[a, y], [b, y]])))
        flip = not flip
    return path


def contours(mask, grid):
    """Per-region [outline, holes..., fill segments...], ordered left to right."""
    xs, ys = grid
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))   # 8-connected, as traced
    res = abs(xs[1] - xs[0])
    out = []
    for k in range(1, n + 1):
        blob = lab == k
        if blob.sum() * res * res < MIN_BLOB_MM2:
            continue
        r, c = np.nonzero(blob)
        out.append((xs[c].mean(), _boundary_loops(blob, xs, ys) + _scanlines(blob, xs, ys)))
    out.sort(key=lambda t: t[0])
    return [segs for _, segs in out]


def _inside(mask, grid, a, b, samples=12):
    """Does the straight segment a->b stay on inked ground?"""
    xs, ys = grid
    t = np.linspace(0, 1, samples)[:, None]
    P = a * (1 - t) + b * t
    ci = np.clip(np.searchsorted(xs, P[:, 0]), 0, len(xs) - 1)
    ri = np.clip(np.searchsorted(-ys, -P[:, 1]), 0, len(ys) - 1)
    return bool(mask[ri, ci].all())


KIND_TRAVEL, KIND_MARK, KIND_APPROACH = 0, 1, 2


def toolpath(mask=None, grid=None):
    """Full path as (N,3) in mm with z relative to the panel surface, plus a
    per-point flag: 0 travel, 1 needle in the work, 2 the slow approach.

    The needle stays down for as long as the connection between two passes
    runs over ink, which is what a machine actually does; it lifts only to
    cross clean skin. Retracting on every scanline would double the cycle time
    and stipple the edges of every dot.

    The plunge lands in two stages. A fast drop to `APPROACH` above the final
    depth, then the last `APPROACH` mm at marking feed.

    A single fast plunge arrives with the loop carrying its full velocity lag,
    v / wn, and the trajectory marks from the first point without waiting: the
    needle enters at a depth that is not the one commanded, at the start of
    every stroke. Splitting it means the last millimetres are travelled at the
    same feed as the drawing itself, so the lag the needle arrives with is the
    lag it would have had anyway.
    """
    if mask is None:
        mask, grid = placeholder_mask()
    pts, kinds = [], []
    down = False

    def plunge(p):
        nonlocal down
        pts.append([p[0], p[1], CLEARANCE]); kinds.append(KIND_TRAVEL)
        pts.append([p[0], p[1], -PLUNGE_DEPTH + APPROACH]); kinds.append(KIND_TRAVEL)
        pts.append([p[0], p[1], -PLUNGE_DEPTH]); kinds.append(KIND_APPROACH)
        down = True

    def retract():
        nonlocal down
        if down:
            last = pts[-1]
            pts.append([last[0], last[1], CLEARANCE]); kinds.append(KIND_TRAVEL)
            down = False

    for region in contours(mask, grid):
        for seg in region:
            if len(seg) == 0:
                continue
            if not down:
                plunge(seg[0])
            else:
                prev = np.array(pts[-1][:2])
                if _inside(mask, grid, prev, seg[0]):
                    pts.append([seg[0, 0], seg[0, 1], -PLUNGE_DEPTH]); kinds.append(KIND_MARK)
                else:
                    retract()
                    plunge(seg[0])
            for p in seg:
                pts.append([p[0], p[1], -PLUNGE_DEPTH]); kinds.append(KIND_MARK)
        retract()
    P = np.array(pts, float)
    kind = np.array(kinds, int)
    # A plunge already places the segment's first point and the fill loop then
    # repeats it. Duplicates are harmless to draw and poison every derivative
    # taken downstream, because a zero length segment takes zero time.
    keep = np.concatenate([[True], np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-9])
    return P[keep], kind[keep]


def lengths(P, kind):
    """(marked, travel) path length in mm, counting each segment once.

    The slow approach counts as travel: it is not on the drawing.
    """
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    cutting = (kind[:-1] == KIND_MARK) & (kind[1:] == KIND_MARK)
    return float(d[cutting].sum()), float(d[~cutting].sum())


# --- self check --------------------------------------------------------------

def _shape(kind, res=0.25, half=20.0):
    """Test masks with known topology, on the same grid convention as the artwork."""
    xs = np.arange(-half, half, res)
    ys = np.arange(-half, half, res)[::-1]
    X, Y = np.meshgrid(xs, ys)
    r = np.hypot(X, Y)
    if kind == "disc":
        m = r <= 15.0
    elif kind == "annulus":
        m = (r <= 15.0) & (r >= 8.0)
    elif kind == "C":
        m = (r <= 15.0) & (r >= 8.0) & ~((X > 0) & (abs(Y) < 4.0))
    elif kind == "E":
        m = (abs(X) <= 10) & (abs(Y) <= 14)
        m &= ~((X > -5) & (abs(abs(Y) - 7) < 3.0))   # notches open to the edge
    elif kind == "B":
        m = (abs(X) <= 10) & (abs(Y) <= 14)
        m &= ~((abs(X) < 5) & (abs(abs(Y) - 7) < 3.0))   # two enclosed counters
    else:
        raise ValueError(kind)
    return m, (xs, ys)


def selftest(verbose=True, report=None):
    """Certify the contour tracer against shapes whose topology is known.

    Three properties, because a tracer can fail any of them on its own:
      closure  - the walk returns to where it started;
      topology - one loop for the outline plus one per hole, no more, no less;
      coverage - every boundary pixel of the region lies on some loop, which is
                 what catches a walk that cuts a thin neck and comes home early.
    """
    # loops expected = outline + holes. A notch that opens to the edge is
    # concave but not a hole, which is the distinction "E" versus "B" pins down.
    cases = [("disc", 1), ("annulus", 2), ("C", 1), ("E", 1), ("B", 3)]
    rows, ok = [], True
    for kind, holes_expected in cases:
        m, (xs, ys) = _shape(kind)
        lab, n = ndimage.label(m, structure=np.ones((3, 3)))
        assert n == 1, f"{kind}: expected one region, got {n}"
        blob = lab == 1
        loops = _loops(blob)

        edge = blob & ~ndimage.binary_erosion(blob)
        traced = {q for loop in loops for q in loop}
        edge_px = set(zip(*(a.tolist() for a in np.nonzero(edge))))
        coverage = len(edge_px & traced) / len(edge_px)

        closed = all(abs(lp[0][0] - lp[-1][0]) <= 1 and abs(lp[0][1] - lp[-1][1]) <= 1
                     for lp in loops)
        good = len(loops) == holes_expected and closed and coverage > 0.999
        ok &= good
        rows.append((kind, holes_expected, len(loops), closed, coverage, good))

    if verbose:
        print("contour tracer")
        print(f"  {'shape':10} {'loops':>11} {'closed':>7} {'coverage':>9}")
        for kind, exp, got, closed, cov, good in rows:
            print(f"  {kind:10} {got:>5} / {exp:<3} {str(closed):>7} "
                  f"{cov * 100:>8.1f}%  {'ok' if good else 'FAIL'}")

    # Perimeter against geometry: a traced 8-connected staircase is longer than
    # the curve it follows, so the smoothed loop is what gets compared.
    m, (xs, ys) = _shape("annulus")
    lab, _ = ndimage.label(m, structure=np.ones((3, 3)))
    segs = _boundary_loops(lab == 1, xs, ys)
    per = [float(np.linalg.norm(np.diff(s, axis=0), axis=1).sum()) for s in segs]
    truth = [2 * np.pi * 15.0, 2 * np.pi * 8.0]
    # Signed, not absolute: the outer loop comes out short and the inner one
    # long, because the trace runs through the centres of the boundary pixels,
    # half a pixel inside the region. An absolute value hid that, and the
    # documents once explained the error as a staircase being longer.
    if report is not None:
        report["annulus_perimeters_mm"] = sorted(per, reverse=True)
        report["annulus_geometry_mm"] = truth
    if verbose:
        for got, want in zip(sorted(per, reverse=True), truth):
            print(f"  perimeter  {got:7.2f} mm   geometry {want:7.2f} mm   "
                  f"{(got - want) / want * 100:+.1f}%")
    ok &= all(abs(g - w) / w < 0.02 for g, w in zip(sorted(per, reverse=True), truth))

    # The official mark, when it has been fetched. Its topology is known by
    # looking at it: nine dots, then R O S, and exactly the R and the O have a
    # counter. Asserting that is what proves the tracer handles real artwork
    # and not only the shapes it was written against.
    if (ARTWORK / "ros_logo.svg").exists():
        mask, (xs, ys) = ros_logo_mask(150.0)
        lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
        res = abs(xs[1] - xs[0])
        regions = [lab == k for k in range(1, n + 1)
                   if (lab == k).sum() * res * res >= MIN_BLOB_MM2]
        holes = sorted(len(_loops(b)) - 1 for b in regions)
        good = len(regions) == 12 and holes == [0] * 10 + [1] * 2
        ok &= good
        if report is not None:
            report["logo_regions"] = len(regions)
            report["logo_counters"] = int(sum(holes))
        if verbose:
            print(f"  ros logo   {len(regions):>2} regions, "
                  f"{sum(holes)} counters  {'ok' if good else 'FAIL'}")
    elif verbose:
        print("  ros logo   not fetched; run fetch_artwork.sh")

    return ok


if __name__ == "__main__":
    if not selftest():
        raise SystemExit("contour tracer self check failed")
    print()
    mask, grid = placeholder_mask()
    P, kind = toolpath(mask, grid)
    marked, travel = lengths(P, kind)
    xs, ys = grid
    print("artwork         : placeholder lattice (NOT the ROS logo)")
    print(f"extents         : {xs[-1] - xs[0]:.0f} x {ys[0] - ys[-1]:.0f} mm")
    print(f"regions         : {len(contours(mask, grid))}")
    print(f"path points     : {len(P)}  ({int((kind == 1).sum())} in the work)")
    print(f"marked length   : {marked:.0f} mm")
    print(f"travel length   : {travel:.0f} mm")
