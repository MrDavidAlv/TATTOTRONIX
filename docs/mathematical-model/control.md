# Dynamics and Control

> Everything here is measured against one tolerance: a tattoo needle lays a line
> about **0.3 mm** wide, and a position error comparable to that is a different
> drawing.

## 1. The dynamic model

$$M(q)\,\ddot q + C(q, \dot q)\,\dot q + G(q) = \tau$$

Computed by **recursive Newton–Euler** for the inverse dynamics
$\tau = \mathrm{ID}(q, \dot q, \ddot q)$, with each link's mass, centre of mass
and inertia tensor read from the same URDF the kinematics comes from:

- $G(q) = \mathrm{ID}(q, 0, 0)$
- column $i$ of $M(q) = \mathrm{ID}(q, 0, e_i) - \mathrm{ID}(q, 0, 0)$, gravity off
- $C(q,\dot q)\dot q = \mathrm{ID}(q, \dot q, 0)$, gravity off

Recovering $M$ from unit-acceleration responses rather than coding a second
algorithm removes the chance of two implementations disagreeing.

Fixed links after the last joint are **lumped** onto `joint_5`'s body, so the pen
mass is felt by that joint instead of quietly disappearing.

### Verification

Newton–Euler gravity torque against the numerically computed gradient of
potential energy, $G_i = \partial U / \partial q_i$, at three poses:

| Pose | Worst error |
|---|---|
| Zero pose | 6.7 × 10⁻¹¹ N·m |
| `q = [0.3, −0.5, 0.7, 0.2, −0.4]` | 7.6 × 10⁻¹¹ N·m |
| `q = [−0.8, 1.0, −0.6, 0.9, 1.1]` | 9.4 × 10⁻¹¹ N·m |

Two independent routes — a rigid body recursion against the derivative of a
scalar — agreeing to machine precision.

### A result that looked like a bug and was not

<div align="center">
<img src="../figures/07_gravity.png" width="900"/>
</div>

At the zero pose `joint_2` and `joint_3` carry **exactly the same** gravity
torque, −0.5424 N·m, even though `joint_2` carries more mass above it.

The reason is geometric: both axes sit at the same $x$ (28.66 mm), and so does
the upper arm's centre of mass, so the upper arm exerts no moment about
`joint_2`. The two joints see identical lever arms for the rest of the chain.

> **Careful with these magnitudes.** The URDF inertias are bounding-box
> approximations at estimated masses. The *structure* of the model is correct;
> the *numbers* are worth whatever those estimates are worth. Any conclusion
> about actuator sizing has to wait for real mass properties from CAD.

## 2. Control structure

Independent joint control, which is what `ros2_control` offers through
`joint_trajectory_controller` and the position interface the description
exposes:

$$\tau = K_p e + K_i \textstyle\int e \, dt + K_d \dot e + \text{(feedforward)}$$

The loop closes at **200 Hz**, the `controller_manager` rate in
`tattotronix_controllers.yaml`. The plant in simulation is the full non-linear
dynamics, integrated at 1 kHz.

### Tuning

Each joint is treated as a second order plant with the inertia taken from the
diagonal of $M(q)$ at a working pose. Fixing critical damping ($\zeta = 1$) and a
bandwidth $\omega_n$:

$$K_p = J\omega_n^2, \qquad K_d = 2\zeta\omega_n J, \qquad K_i = 0.15\,\omega_n K_p$$

What matters is not the values but that **every gain traces back to a measured
inertia and a single bandwidth decision**, rather than to a knob someone turned
until it stopped oscillating. When the real masses arrive, the gains recompute
themselves.

<div align="center">
<img src="../figures/08_step.png" width="900"/>
</div>

## 3. The velocity lag

For a second order loop of bandwidth $\omega_n$ following a constant feed $v$,
the steady state error is

$$e_{\text{lag}} \approx v / \omega_n$$

At 6 mm/s and 20 rad/s that is 300 µm — the whole line width. **The error does
not depend on how well the loop is tuned, but on being asked to produce the
velocity out of the error.** There are two ways out and they are not equivalent:
raise $\omega_n$, which is bounded by the 200 Hz rate and by torque, or feed the
reference forward, so the loop no longer has to generate the velocity.

### How the error is measured, and why that matters

<div align="center">
<img src="../figures/09_tracking.png" width="900"/>
</div>

Averaging the error over the whole window is **not a number about the drawing**.
Travel runs at 60 mm/s, ten times the marking feed, so its lag is ten times
larger; a joint mean ends up reporting *how much travel happened to fall inside
the window*. Only the marking error ever touches skin.

Split, over the first 12 s of the logo — one dot, filled without lifting:

| Segment | Mean | Worst |
|---|---|---|
| Marking, settled | **131.7 µm** | 2717 µm |
| Marking, all | 265.5 µm | 7639 µm |
| Travel | 3219 µm | 8402 µm |

The split was validated before the artwork changed: on the stand-in it gave
164.5 µm settled, against the 165 µm `control_study.py` obtains independently for
the same structure and bandwidth. Two routes, one number.

## 4. The needle enters before the loop settles

Splitting the segments exposes what the mean was hiding: **the worst marking
error is 7.6 mm**, and it sits immediately after each plunge.

The plunge is a travel move: the reference drops 9.5 mm at 60 mm/s and stops
dead. Arriving at the bottom the loop is carrying $v/\omega_n = 3$ mm of lag, and
the trajectory **marks from the first point**, without waiting. On skin that is a
needle entering at a depth that is not the one commanded, at the start of every
stroke.

Dropping one second-order settling time, $4/\omega_n = 200$ ms, after each entry
takes the worst from 7639 to 2717 µm — that is, **it is not enough**. In the
figure the ringing lasts around 0.8 s, four times what second order predicts,
because the reference stops dead and the integrator arrives wound up.

This is not corrected with a more generous window, which would be choosing the
number. It is a defect in the **trajectory**, not in the metric. The fixes are to
decelerate the approach over the last millimetre, or to wait for the settle
before marking.

**With the real logo this stops being a footnote.** The stand-in lifted the
needle twice; the logo lifts it **98 times**, because every fill pass that meets
the counter of the R or the O has to exit and re-enter. Ninety-eight stroke
starts, each carrying this transient.

## 5. What actually helps

<div align="center">
<img src="../figures/10_control_study.png" width="900"/>
</div>

Measured over the **busiest 12 s of the path** — the stretch with the most needle
entries, found from the path rather than fixed, which lands inside the **O** with
7 entries. Running the study on the opening dots would measure the easy part of
the drawing.

At $\omega_n = 20$ rad/s, full non-linear plant:

| Structure | Settled, marking | Worst, marking | Travel | Peak torque |
|---|---|---|---|---|
| PID only | 1926 µm | 48 593 µm | 3777 µm | 2.38 N·m |
| PID + gravity | 1123 µm | 11 745 µm | 3855 µm | 1.98 N·m |
| PID + gravity + velocity | 951 µm | 9968 µm | 3643 µm | 1.90 N·m |
| Computed torque | **844 µm** | 7111 µm | 3365 µm | 1.85 N·m |

Bandwidth sweep, PID + gravity:

| $\omega_n$ | Settled, marking | Worst, marking | Peak torque | |
|---|---|---|---|---|
| 10 rad/s | 2943 µm | 15 858 µm | 1.81 N·m | |
| 20 rad/s | 1123 µm | 11 745 µm | 1.98 N·m | |
| 40 rad/s | **193 µm** | 2543 µm | 2.33 N·m | |
| 80 rad/s | — | — | — | **diverges** |
| 120 rad/s | — | — | — | **diverges** |

### What holds and what does not

1. **Gravity feedforward is free and still worth having** (1926 → 1123 µm).
2. **Velocity feedforward still lowers error and torque together** (1123 → 951 µm,
   1.98 → 1.90 N·m). The loop stops fighting its own reference.
3. **Computed torque no longer merely ties — it wins** (844 µm, best of the four).
   On the stand-in it did not beat velocity feedforward, and the reason given was
   that the reference acceleration comes from finite differences on a path with
   discontinuous velocity. That is still true, but the logo has so many
   discontinuities that the acceleration term, noisy as it is, corrects more than
   it injects. *The earlier conclusion was not wrong: it was measured on a drawing
   that did not have them.*
4. **The stability cliff is where it was.** Between 40 and 80 rad/s, set by the
   200 Hz `controller_manager` rate, not by the gains. Raising it needs a faster
   controller, not different tuning.

**And what none of the four touches:** the worst-case column. Those are the needle
entries, and they stay in the thousands of microns in all four. Control does not
fix a trajectory defect.

## 6. Recommended configuration

$\omega_n = 40$ rad/s, PID by pole placement on the effective inertia, with
gravity and velocity feedforward. Over the same hard window:

| Configuration | Settled, marking | Worst, marking | Peak torque |
|---|---|---|---|
| $\omega_n = 20$, PID + g + v | 951 µm | 6696 µm | 1.90 N·m |
| $\omega_n = 40$, PID + g | **193 µm** | 2543 µm | 2.33 N·m |
| $\omega_n = 40$, PID + g + v | 206 µm | **2335 µm** | **2.02 N·m** |

The third is recommended. Its mean is 7% worse than the second, inside the noise
of a 12 s window, but its worst case is better and peak torque drops **13%**,
2.33 to 2.02 N·m. Against a 20 N·m limit neither is tight, but margin is spent
once and the feedforward is free.

### The price of no longer measuring the easy part

This model used to quote **22.5 µm mean and 272 µm worst**, and concluded it was
the first configuration that could draw the logo recognisably. That figure was
measured on continuous marking, on a stretch with no needle lifts. It is correct
for what it measures — the **steady state**. It is not what the logo asks for.

On the hard stretch the same configuration gives **206 µm mean** against a 0.3 mm
line — a factor of 1.5, not 13 — and a worst case of **2.3 mm**, eight times the
line width.

The honest conclusion is that **control is no longer the problem, and is also not
the solution**. In steady state there is margin to spare. What is missing is for
the trajectory to stop driving the needle into the work mid-deceleration, 98
times per drawing.
