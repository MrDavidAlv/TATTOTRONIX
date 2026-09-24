# Mathematical Model of the TATTOTRONIX Manipulator

Model of a **five axis desktop manipulator** that draws the official ROS logo on
a flat panel. It covers kinematics, the artwork-to-toolpath pipeline, rigid body
dynamics, joint control and the error budget that ties them together.

> **Every number here is produced by a script, not typed.** The scripts live in
> [`docs/scripts/`](../scripts/) and read the same URDF that
> `robot_state_publisher` loads, so the model cannot drift away from the robot:
>
> ```bash
> docs/scripts/fetch_artwork.sh    # the official ROS logo, CC BY-NC, not vendored
> python3 docs/scripts/analysis.py
> python3 docs/scripts/figures.py
> ```
>
> Where a value is an estimate rather than a measurement, it says so. That
> distinction is the point of the document: a detailed placeholder invites
> people to measure off it.

<div align="center">
<img src="../figures/04_toolpath.png" width="900"/>
</div>

---

## Contents

1. **[Kinematics](./kinematics.md)** — the chain, forward kinematics verified
   against live TF, the task Jacobian, and **why five axes are exactly enough
   for this job rather than one short of six**
2. **[Toolpath](./toolpath.md)** — artwork to ink mask to contour and fill,
   contour tracing, and **why the stand-in artwork hid two separate defects**
3. **[Control](./control.md)** — tuning by pole placement, the velocity lag,
   the needle entry transient that no control structure fixes, and **why the
   stability cliff was never about the gains**
4. **[Parameters](./parameters.md)** — every value, with its source

---

## Notation

### Frames

| Symbol | Description | ROS 2 frame |
|---|---|---|
| $\{0\}$ | Arm base, on the bench top | `base_link` |
| $\{F\}$ | Tool flange, $+z$ along the tool axis | `tool0` |
| $\{T\}$ | Needle tip, where paths are planned | `tattoo_tcp` |
| $\{P\}$ | Panel, $(u, v)$ in the drawing plane | — |

In Gazebo the bench is 750 mm tall, so the arm's $z = 0$ is the world's
$z = 0.75$.

### Symbols

| Symbol | Meaning | Units |
|---|---|---|
| $q \in \mathbb{R}^5$ | Joint angles | rad |
| $T_0^T(q)$ | Tip pose from the base | — |
| $J(q) \in \mathbb{R}^{5 \times 5}$ | Task Jacobian: 3 position rows, 2 axis-tilt rows | — |
| $\sigma_1 \dots \sigma_5$ | Singular values of $J$, descending | — |
| $M(q), C(q,\dot q), G(q)$ | Inertia, Coriolis, gravity | kg·m², N·m·s, N·m |
| $\omega_n$ | Closed loop bandwidth per joint | rad/s |
| $w$ | Tattoo line width, the tolerance everything is judged against | 0.3 mm |

### Units

Metres and radians in the code. Millimetres and degrees in this document
wherever they read better.

---

## The one number that matters

A tattoo needle lays a line about **0.3 mm** wide. A position error comparable to
that is not noise, it is a different drawing. Every tolerance in these documents
is quoted against it.

| Error source | Magnitude | Against 0.3 mm | Status |
|---|---|---|---|
| Worst tracking error | 46.8 µm, at the hardest moment of the drawing | 0.16× | **Still the largest modelled term.** Bounded by the 1 kHz control rate |
| Settled tracking error | 6.5 µm | 0.02× | Bounded by the 1 kHz control rate |
| Path discretisation | chord error below the mask resolution | — | **Fixed** by a 0.15 mm step — see [toolpath](./toolpath.md#what-the-resampling-step-is-actually-for) |
| Needle entry transient | was 3.2 mm, 98 times per drawing | — | **Fixed** by a 4 mm slow approach — see [control](./control.md#4-the-needle-enters-before-the-loop-settles) |
| Inverse kinematics residual | $10^{-6}$, dimensionless | — | Negligible |
| Servo resolution | not modelled | — | Needs the encoder |
| Backlash and flexure | not modelled | — | Needs the hardware |
| Tissue deformation | not modelled | — | A separate project |

The table is sorted by magnitude, and **the order is the result**. Every modelled
term is now well inside the line width — the worst instant of the drawing sits at
a sixth of it.

What is left is the bottom three rows, which are not modelled at all. They are
what stops this being a credible *accuracy* figure rather than a credible
*control* figure.
