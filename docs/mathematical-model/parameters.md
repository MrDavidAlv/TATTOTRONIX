# Parameter Reference

Every value with its source. **Measured** means read off the CAD mesh or the
URDF. **Derived** means computed by a script from measured values. **Estimated**
means a placeholder standing in for something not yet measured — those are the
rows that bound what this model can honestly claim.

---

## Geometry

| Parameter | Value | Source |
|---|---|---|
| Joint origins, all five | see [kinematics](./kinematics.md#1-the-chain) | **Measured** — `tattotronix.urdf.xacro` |
| Joint limits | ±1.57 rad, all axes | **Measured** — URDF |
| `tool0` offset from `tool_mount_link` | `0.029, -0.012064, 0.005489` m, rpy `0, π/2, 0` | **Measured** off the bracket mesh — see README, *The tool mount* |
| `tool0` → `tattoo_tcp` | `0, 0, 0.045` m | **Estimated** — the pen is one cylinder until it is built |
| Tip position at zero pose | `258.96, −9.44, 245.58` mm | **Derived**, verified against live TF |
| Bench height in Gazebo | 750 mm | **Measured** — world file |

## Panel and artwork

| Parameter | Value | Source |
|---|---|---|
| Panel centre | $x$ = 210 mm, surface at $z$ = +5 mm | **Measured** — world file |
| Panel size | 200 × 140 mm | **Measured** — world file |
| Panel reachable fraction | 98.4% | **Derived** — `analysis.py` |
| Logo width on the panel | 150 mm | **Derived** — largest size keeping the condition number in the thirties, see [kinematics](./kinematics.md#a-correction-worth-recording) |
| Logo offset in $x$ | 0 mm, centred | **Derived** — $\sigma_5$ falls as the drawing moves out |
| Mask resolution | 0.25 mm | Chosen — below the 0.3 mm line width |
| Rasterisation | 2× then downsampled | Required — at 1× the R and O counters close |

## Process

| Parameter | Value | Source |
|---|---|---|
| Line width | 0.3 mm | **Estimated** — the tolerance everything is judged against |
| Stroke pitch | 1.2 mm | Chosen — overlap on a 0.3 mm line |
| Path resampling | 0.15 mm | **Derived** — swept in `resample_study.py`; the reference the velocity feedforward differentiates |
| Plunge depth | 1.5 mm below surface | **Estimated** — depends on tissue |
| Clearance height | 8 mm | Chosen — clears the panel with margin |
| Slow approach | 4 mm | **Derived** — swept in `approach_study.py`; removes most of the entry transient, and at the recommended bandwidth the worst entry is inside the line |
| Marking feed | 6 mm/s | **Estimated** — the order of a tattooist's hand |
| Travel feed | 60 mm/s | Chosen — limited by the arm, not the process |

## Path, on the logo

| Quantity | Value |
|---|---|
| Path points | 16 656 |
| Regions | 12 — nine dots, R, O, S |
| Counters | 2 — the R and the O |
| Marked length | 2558 mm |
| Travel length | 3455 mm |
| Needle entries | **117** |
| Total time | 557 s |
| IK convergence | 100% |
| IK worst residual | 1.0 × 10⁻⁶ |

## Conditioning

| Quantity | Value over the path |
|---|---|
| $\sigma_1$ | 1.7485 … 1.7573 |
| $\sigma_5$ | 0.0483 … 0.0846 |
| Condition number | 21 … 36 |
| Manipulability | 6.21 × 10⁻⁴ … 3.66 × 10⁻³ |

## Dynamics

| Parameter | Value | Source |
|---|---|---|
| Link volumes | integrated from each mesh | **Measured** — closed surfaces, see `mass_properties.py` |
| Print density | 0.35 of solid PLA, 1240 kg/m³ | **Declared** — a 20% infill part, which is not 20% of solid |
| Servos | five MG996R at 55 g, two SG90 at 9 g | **Declared** — catalogue figures; the models were not confirmed |
| Link masses and inertia tensors | `inertials_printed.xacro` | **Derived** — each printed shell plus the servos mounted in it; see [mass properties](./mass.md) |
| Peak gravity torque on the path | 0.31 N·m | **Derived** from the mass model above |
| Peak commanded torque | 0.12 N·m | **Derived** |
| Joint effort limit | 20 N·m, all axes | **Placeholder** — far above the servos the arm carried: an MG996R is rated at roughly 1 N·m and an SG90 at roughly 0.2 N·m |
| Joint velocity limit | 1.5 rad/s, all axes | **Estimated** — one number for every axis |

> The structure of the model is verified to machine precision
> ([control](./control.md#verification)). The magnitudes rest on two
> declarations, the print density and the servo figures, because the arm they
> describe was lost and cannot be weighed. Everything else in them is measured.
> Weighing one printed link and one servo, once the arm is rebuilt, replaces
> both.

## Control

| Parameter | Value | Source |
|---|---|---|
| Control rate | 1 kHz | **Derived** — swept in `rate_study.py`; it is what sets the bandwidth ceiling |
| Simulation integration | 1 kHz, explicit Euler | Chosen |
| Recommended bandwidth $\omega_n$ | 160 rad/s | **Derived** — usable up to about a quarter of the loop rate |
| Stability boundary, $\omega_n T$ | 0.316 at the tuning pose; 0.302 to 0.345 along the drawing | **Derived** — $2/(3\lambda_{\max})$, see [control](./control.md#7-the-rate-is-the-ceiling) |
| Damping $\zeta$ | 1.0 | Chosen — critical |
| $K_p$ | `4.322, 3.196, 0.7526, 0.0218, 0.0362` | **Derived** at $\omega_n$ = 20 |
| $K_i$ | `28.82, 21.31, 5.017, 0.1456, 0.2414` | **Derived** at $\omega_n$ = 20 |
| $K_d$ | `0.2161, 0.1598, 0.03763, 0.00109, 0.00181` | **Derived** at $\omega_n$ = 20 |
| Feedforward | gravity + velocity | **Derived** — see [control](./control.md#6-recommended-configuration) |

## Performance, on the hardest stretch

| Metric | Value | Against a 0.3 mm line |
|---|---|---|
| Settled marking error | 6.5 µm | 0.02× |
| Worst marking error | 46.8 µm | 0.16× |
| Peak torque | 0.35 N·m | 2% of the 20 N·m limit |

Both are well inside the line width — for the joint loop this models. The servos
the arm was built with cannot hold a line this fine at all: their dead band alone
leaves the needle up to 3.80 mm off it; see [actuators](./actuators.md). The
torque share is against the
description's 20 N·m placeholder, not against the servos the arm carried, which
are rated at roughly 1 N·m for the MG996R and 0.2 N·m for the SG90; measured
against those, the same peak is a substantial fraction of what the motors can
give, and sizing the actuators properly is the next piece of work. Before the slow approach was added they
were 284.9 µm and 3221 µm, the second of them 11 times the line; before the
loop rate was raised, 26.6 µm and 338.0 µm.

---

## Actuators

| Parameter | Value | Source |
|---|---|---|
| Dead band, MG996R | 5 µs | **Declared** — catalogue |
| Dead band, SG90 | 10 µs | **Declared** — catalogue |
| Pulse per degree | 11.1 µs | **Declared** — the 500 to 2500 µs convention for a 180° servo; measure it |
| Command step, PCA9685 at 50 Hz | 4.88 µs | **Derived** — a 20 ms frame in 4096 counts |
| Command step, Arduino `Servo` | 1 µs | **Derived** — whole microseconds |
| Bus servo resolution | 0.088° | **Declared** — one count of a 12-bit encoder |
| Joint resolution a 0.3 mm line needs | 0.037° | **Derived** — see [actuators](./actuators.md#5-what-the-drawing-needs) |

## What this model cannot tell you

Two things are not modelled at all, and with the servo resolution above they are
what stop this from being a credible accuracy figure rather than a credible
*control* figure:

| Missing | Why it matters | What it needs |
|---|---|---|
| Backlash and flexure | A printed bracket and an SG90 have both; each 0.1° of play costs up to 0.81 mm | Measuring the hardware |
| Tissue deformation | The panel is rigid; skin is not | A separate project |

The model says what the control does. It does not say what an SG90 with backlash
does in a printed bracket.
