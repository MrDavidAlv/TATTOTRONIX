# Mathematical Model of the TATTOTRONIX Manipulator

Model of a **five axis desktop manipulator** that draws the official ROS logo on
a flat panel. It covers kinematics, the artwork-to-toolpath pipeline, rigid body
dynamics, joint control and the error budget that ties them together.

> **Every number here comes from a script, and a check holds the documents to
> it.** The scripts live in [`docs/scripts/`](../scripts/) and read the same URDF
> that `robot_state_publisher` loads; their results land in `docs/data/`, and
> `docs/scripts/check_docs.py` fails the build if a published figure no longer
> matches them. That second half is recent, and it is what found the figures
> that had been typed and had gone stale.
>
> ```bash
> docs/scripts/fetch_artwork.sh              # the official ROS logo, CC BY-NC, not vendored
> python3 docs/scripts/mass_properties.py --write-xacro --write-data
> python3 docs/scripts/analysis.py           # 2 min
> python3 docs/scripts/kinematics_study.py   # 4 min
> python3 docs/scripts/control_study.py      # 15 min; also approach_, resample_, rate_study.py
> python3 docs/scripts/actuator_study.py     # seconds
> python3 docs/scripts/figures.py
> python3 docs/scripts/check_docs.py
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
4. **[Mass properties](./mass.md)** — volumes integrated from the meshes, the
   arm as built as printed shells plus seven servos, and **why the box
   approximation made it eight times too heavy**
5. **[Actuators](./actuators.md)** — what hobby servos let any controller do,
   and **why the servos, not the control, now set the floor under the error**
6. **[Parameters](./parameters.md)** — every value, with its source

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
| Servo resolution, as built | 3.80 mm worst, 1.43 mm typical, driven by a PCA9685 | 12.7× | **The largest term by far**, and the servos' own: no controller above them removes it — see [actuators](./actuators.md) |
| Worst tracking error | 46.8 µm, at the hardest moment of the drawing | 0.16× | **The largest term the controller owns.** Bounded by the 1 kHz control rate |
| Path discretisation | 45.1 µm chord error at the 0.15 mm step | 0.15× | Under the 0.25 mm mask pitch — see [toolpath](./toolpath.md#what-the-resampling-step-is-actually-for) |
| Settled tracking error | 6.5 µm | 0.02× | Bounded by the 1 kHz control rate |
| Needle entry transient | was 3.2 mm, 117 times per drawing | — | **Fixed** by a 4 mm slow approach together with the recommended bandwidth — see [control](./control.md#4-the-needle-enters-before-the-loop-settles) |
| Inverse kinematics residual | 1.0 × 10⁻⁶ | — | Negligible |
| Actuator torque | modelled against a 20 N·m placeholder | — | **Not credible yet**: the servos the arm carried are rated near 1 N·m and 0.2 N·m |
| Backlash and flexure | not measured; up to 0.81 mm per 0.1° of play at every joint | — | Needs the hardware — see [actuators](./actuators.md#5-what-the-drawing-needs) |
| Tissue deformation | not modelled | — | A separate project |

The table is sorted by magnitude, and **the order is the result**. Everything the
controller owns is well inside the line width — the worst instant of the drawing
sits at a sixth of it. The servos are not: the arm as built can come to rest 12.7
line widths from where it was sent, whatever the controller does, and bus servos
with encoders on every joint would still leave 2.4. What limits the drawing now
is the hardware, not the control.

Below that, one term is modelled against a placeholder and two are not modelled
at all.
