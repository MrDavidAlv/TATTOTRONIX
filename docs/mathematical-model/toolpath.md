# From Artwork to Toolpath

> The pipeline is mask driven on purpose. Anything that can be turned into a
> binary ink mask goes through the same contour, fill, ordering and lead-in code.
> Changing the drawing changes one function and nothing else.

## 1. The pipeline

```
   image (PNG/SVG)  ──►  binary mask  ──►  connected components
                                                  │
                     ┌────────────────────────────┘
                     ▼
         contour pass + zigzag fill  ──►  ordering, lead-ins and lifts
                     │
                     ▼
         path (u, v, z) in mm on the panel, with a needle-down flag
```

## 2. The artwork

The mark is the **official ROS logo**, rasterised from the SVG published by
[`ros-infrastructure/artwork`](https://github.com/ros-infrastructure/artwork).

It is **fetched, not vendored**. The mark is CC BY-NC 4.0 and covered by the
[ROS trademark policy](https://www.ros.org/blog/media/); this repository is
Apache-2.0, and a non-commercial licence cannot ride along inside an Apache-2.0
tree.

```bash
docs/scripts/fetch_artwork.sh      # lands in docs/artwork/, which is ignored
```

**Rasterised at twice the sampling pitch, then downsampled.** At one subpixel per
cell the counters of the **R** and the **O** close up at working sizes, turning
two holes into none with nothing to warn you. At 2× each edge pixel is decided by
four subpixels.

### What it replaced

Development ran for a long time against a **declared stand-in**: a 3×3 dot
lattice with roughly the right character. Reproducing the logo from memory would
have been invented geometry presented as official, which is the one mistake this
project avoids everywhere else.

Swapping it in changed **nothing downstream** — contour, fill, ordering,
lead-ins, IK and control all ran on the real mark without a line changed outside
the artwork loader. That was the design goal, and it is now demonstrated rather
than asserted.

It changed the **results** considerably. The stand-in was nine convex blobs; the
logo has two counters and **98 needle lifts against 2**. That jump is what
exposed the needle entry transient as the dominant error
([control](./control.md#4-the-needle-enters-before-the-loop-settles)). *The
stand-in was hiding two separate defects, not one.*

## 3. Contour tracing

The contour pass has to walk a region's boundary **in order**.

The first implementation sorted boundary pixels by their angle about the
centroid. That is a correct outline exactly when the region is convex and has no
hole. The stand-in's dots are discs, so it worked — **the stand-in was hiding the
defect.**

It fails on any letterform. A concave shape has several boundary pixels at the
same angle, and sorting by angle chains them into a zigzag between the inner and
outer edge: a star, not a contour. A counter is invisible to it, because its
pixels land in the same list as the exterior's.

<div align="center">
<img src="../figures/11_contours.png" width="900"/>
</div>

### Moore neighbour tracing

Stand on a boundary pixel, sweep the eight neighbours clockwise starting from
wherever you came in, and step onto the first filled one.

**Jacob's stopping criterion** ends the walk on re-entering the start pixel *from
the same neighbour*. Stopping merely on reaching the start again cuts the loop
short at any one-pixel neck, which is what serifs and thin strokes are made of.

Holes come from labelling the background: background components that do not touch
the frame are counters, and each is traced from the filled pixel immediately
above its top-left pixel. Foreground is labelled **8-connected** to match the
tracer and background **4-connected**, the pairing that stops a diagonal pixel
bridge from reading solid on one side and open on the other.

The pixel staircase comes off with a closed `[1 2 1]/4` pass. Each pass moves a
point by at most half a grid step, so at 0.25 mm the whole correction stays under
the 0.3 mm line width, and it does not walk the boundary inwards the way an
erosion would.

### Certification

`rospath.py` checks itself on every run, against shapes whose topology is known,
on three properties a tracer can fail independently:

| Shape | What it tests | Loops expected | Loops | Closed | Boundary coverage |
|---|---|---|---|---|---|
| disc | convex, no hole | 1 | 1 | yes | 100% |
| annulus | one hole | 2 | 2 | yes | 100% |
| C | concave | 1 | 1 | yes | 100% |
| E | notches open to the edge | 1 | 1 | yes | 100% |
| B | two enclosed counters | 3 | 3 | yes | 100% |
| **ROS logo** | **real artwork** | **12 regions, 2 counters** | **12 / 2** | yes | — |

**Coverage is the property that matters.** It is what catches a walk that cuts
through a neck and comes home early; a loop can be closed and have the right hole
count and still have missed half the boundary.

Annulus perimeter against geometry: **93.48 mm** vs 94.25 mm outside (+0.8%) and
**50.92 mm** vs 50.27 mm inside (+1.3%). The residual is the pixel staircase the
smoothing does not quite erase, and it errs in the expected direction — an
8-connected staircase is longer than the curve it follows.

The `E` / `B` pair pins down the distinction that matters: a notch open to the
edge is concave but **not** a hole.

## 4. Fill, not outline

A tattoo machine does not draw lines, it **packs**. The fill is a boustrophedon
at the needle's stroke pitch, preceded by one pass around the boundary, which is
how a rotary machine lines and then shades a solid.

The needle stays down as long as the connection between two passes runs over ink,
and lifts only to cross clean skin. An earlier version retracted on *every* fill
pass, which doubled cycle time and would have stippled every edge.

<div align="center">
<img src="../figures/04_toolpath.png" width="900"/>
</div>

<div align="center">
<img src="../figures/drawing.gif" width="900"/>
<br/>
<sub>The same path, run. 537 s compressed into 24.</sub>
</div>

| Parameter | Value | Reason |
|---|---|---|
| Stroke pitch | 1.2 mm | Overlap on a ~0.3 mm line |
| Resampling | 0.15 mm | See below — it is the reference the feedforward differentiates, not just spacing |
| Depth | 1.5 mm below the surface | **Placeholder** — depends on tissue |
| Clearance | 8 mm | Clears the panel with margin |
| Slow approach | 4 mm | The last part of the plunge, at marking feed — see [control](./control.md#the-fix) |
| Marking feed | 6 mm/s | The order of what a tattooist does |
| Travel feed | 60 mm/s | Limited by the arm, not the process |

**On the logo:** 16 732 points, 2558 mm marked against 3530 mm travelled, 561 s
total, over **121 needle entries**. Every fill pass that meets the counter of the
R or the O has to lift and re-enter, and that is where the travel goes.

That last number used to be the project's dominant error, because each entry
drove the needle in while the loop was still settling. The
[slow approach](./control.md#the-fix) fixed that, at 14% of cycle time. Ordering
the passes to lift less would give that time back, and is worth doing for that
reason rather than for accuracy now.

## 5. Timing and sampling

Constant feed per move type. Each segment's time is its length over its feed, and
the cumulative time gives the reference $q_{\text{ref}}(t)$ the controller
consumes.

### What the resampling step is actually for

<div align="center">
<img src="../figures/13_resample.png" width="900"/>
</div>

The error budget used to list "0.6 mm between points" against a 0.3 mm line and
call it twice the line width. That conflates the *spacing* of the points with
the *error* they cause. The controller interpolates between them, so what the
spacing costs is chord deviation — how far the straight run between two points
departs from the curve they were sampled from — which goes as roughly
$h^2/8R$, not as $h$. Measured on the traced contours it is **118 µm** at
0.6 mm, not 600.

It still mattered, for a different reason. The reference velocity handed to the
feedforward comes from finite differences on this path, so a coarse step makes
that velocity a staircase and the feedforward injects as much as it corrects.

| Step | Points | Chord error | Settled | Worst, marking | Peak torque |
|---|---|---|---|---|---|
| 0.60 mm | 4470 | 118.0 µm | 19.3 µm | 178.9 µm | 2.16 N·m |
| 0.30 mm | 8568 | 73.4 µm | 24.3 µm | 189.0 µm | 1.94 N·m |
| **0.15 mm** | 16 732 | below the mask | **7.9 µm** | **19.4 µm** | **1.68 N·m** |

**Peak torque falls as the step is refined**, which is the giveaway: a finer path
is not asking the arm for more, it is asking for something smoother. Inverse
kinematics still converges at 100% of points and the whole cycle grows by 4%.

Two honest notes. The 0.30 mm row is slightly *worse* than 0.60 on the dynamic
figures — the trend is not monotonic at this resolution and the table says so
rather than drawing a clean curve through it. And "below the mask" is not zero
error: the traced contour is only known to the 0.25 mm mask pitch, so at 0.15 mm
there is no finer detail left to depart from. Going below this buys nothing
until the mask resolution goes with it.

### Time parameterisation

This is deliberately simple. The plunge, which was the worst case of that
simplicity, is handled specially by the slow approach above. Every other segment
still starts and stops in a step, which is why acceleration feedforward has
nothing clean to work with (see [control](./control.md#5-what-actually-helps));
a trapezoidal profile with bounded acceleration is the general version of what
the slow approach does for one move.
