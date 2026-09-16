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
| Slow approach | 4 mm | **Derived** — swept in `approach_study.py`; where the worst needle entry falls under the line width |
| Marking feed | 6 mm/s | **Estimated** — the order of a tattooist's hand |
| Travel feed | 60 mm/s | Chosen — limited by the arm, not the process |

## Path, on the logo

| Quantity | Value |
|---|---|
| Path points | 16 732 |
| Regions | 12 — nine dots, R, O, S |
| Counters | 2 — the R and the O |
| Marked length | 2558 mm |
| Travel length | 3530 mm |
| Needle entries | **121** |
| Total time | 561 s |
| IK convergence | 100% |
| IK worst residual | 1.0 × 10⁻⁶ |

## Conditioning

| Quantity | Value over the path |
|---|---|
| $\sigma_1$ | 1.7486 … 1.7573 |
| $\sigma_5$ | 0.0484 … 0.0846 |
| Condition number | 21 … 36 |
| Manipulability | 6.21 × 10⁻⁴ … 3.66 × 10⁻³ |

## Dynamics

| Parameter | Value | Source |
|---|---|---|
| Link masses | see URDF | **Estimated** — bounding-box approximations |
| Link inertia tensors | see URDF | **Estimated** — bounding-box approximations |
| Peak gravity torque on the path | 1.91 N·m | **Derived** from the estimates above |
| Peak commanded torque | 0.79 N·m | **Derived** |
| Joint effort limit | 20 N·m, all axes | **Estimated** — one number for every axis |
| Joint velocity limit | 1.5 rad/s, all axes | **Estimated** — one number for every axis |

> Everything in this section inherits the uncertainty of the mass estimates. The
> structure of the model is verified to machine precision
> ([control](./control.md#verification)); the magnitudes are worth whatever a
> bounding box is worth.

## Control

| Parameter | Value | Source |
|---|---|---|
| Control rate | 1 kHz | **Derived** — swept in `rate_study.py`; it is what sets the bandwidth ceiling |
| Simulation integration | 1 kHz, explicit Euler | Chosen |
| Recommended bandwidth $\omega_n$ | 160 rad/s | **Derived** — usable up to about a quarter of the loop rate |
| Damping $\zeta$ | 1.0 | Chosen — critical |
| $K_p$ | `32.24, 19.43, 5.84, 0.252, 0.096` | **Derived** at $\omega_n$ = 20 |
| $K_i$ | `214.9, 129.5, 38.9, 1.68, 0.637` | **Derived** at $\omega_n$ = 20 |
| $K_d$ | `1.612, 0.971, 0.292, 0.0126, 0.0048` | **Derived** at $\omega_n$ = 20 |
| Feedforward | gravity + velocity | **Derived** — see [control](./control.md#6-recommended-configuration) |

## Performance, on the hardest stretch

| Metric | Value | Against a 0.3 mm line |
|---|---|---|
| Settled marking error | 6.3 µm | 0.02× |
| Worst marking error | 36.3 µm | 0.12× |
| Peak torque | 2.37 N·m | 12% of the 20 N·m limit |

Both are well inside the line width. Before the slow approach was added they
were 205.7 µm and 2335 µm, the second of them eight times the line; before the
loop rate was raised, 15.2 µm and 200.0 µm.

---

## What this model cannot tell you

Three things are not modelled at all, and they are what stop this from being a
credible accuracy figure rather than a credible *control* figure:

| Missing | Why it matters | What it needs |
|---|---|---|
| Servo resolution | Sets the smallest commandable motion | The encoder |
| Backlash and flexure | A printed bracket and an SG90 have both | Measuring the hardware |
| Tissue deformation | The panel is rigid; skin is not | A separate project |

The model says what the control does. It does not say what an SG90 with backlash
does in a printed bracket.
