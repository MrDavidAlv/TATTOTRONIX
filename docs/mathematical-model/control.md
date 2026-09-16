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
| Marking, settled | **94.0 µm** | 720 µm |
| Marking, all | 99.1 µm | 720 µm |
| Travel | 1313 µm | 3671 µm |

The split was validated before the artwork changed: on the stand-in it gave
164.5 µm settled, against the 165 µm `control_study.py` obtains independently for
the same structure and bandwidth. Two routes, one number.

## 4. The needle enters before the loop settles

> **Fixed.** This section records the defect, how it was measured and what it
> cost, because the fix is only convincing next to the thing it fixed.

### The defect

Splitting the error by needle state exposed what the mean was hiding: the worst
marking error was **7.6 mm**, and it sat immediately after each plunge.

The plunge was a travel move. The reference dropped 9.5 mm at 60 mm/s and
stopped dead on the surface. Arriving at the bottom the loop was carrying
$v/\omega_n = 3$ mm of lag, and the trajectory marked from the first point,
without waiting. On skin that is a needle entering at a depth that is not the
one commanded, at the start of every stroke.

Dropping one second-order settling time, $4/\omega_n = 200$ ms, after each entry
took the worst from 7639 to 2717 µm — that is, **it was not enough**. The ringing
lasted around 0.8 s, four times what second order predicts, because the
reference stopped dead and the integrator arrived wound up.

And with the real logo it was not a footnote. The stand-in artwork lifted the
needle twice; the logo lifts it **98 times**, because every fill pass that meets
the counter of the R or the O has to exit and re-enter.

### The fix

Land the last few millimetres at the marking feed instead of the travel feed.
The plunge becomes two stages: a fast drop to `APPROACH` above the final depth,
then the rest at 6 mm/s. The needle then arrives carrying the lag it would have
had anyway while drawing, rather than ten times it.

It is a **trajectory** fix, not a control fix, which is why none of the four
control structures had touched it.

<div align="center">
<img src="../figures/12_approach.png" width="900"/>
</div>

Swept with everything else held fixed, every run starting at the same needle
entry:

| Slow approach | Worst entry | Settled mean | Peak torque | Cycle time |
|---|---|---|---|---|
| 0.0 mm | 2335 µm | 205.7 µm | 2.02 N·m | 470 s |
| 0.5 mm | 1963 µm | 97.0 µm | 1.96 N·m | 486 s |
| 1.0 mm | 1243 µm | 78.1 µm | 2.01 N·m | 493 s |
| 2.0 mm | 518 µm | 37.4 µm | 2.09 N·m | 508 s |
| **4.0 mm** | **196 µm** | 26.2 µm | 2.17 N·m | 537 s |

**4 mm is where the worst entry falls under the 0.3 mm line width.** It costs
14% of cycle time and essentially no torque.

Two honesty notes on that table. The window is 12 s of path, and slowing the
path means fewer entries fall inside it — seven at 0 mm, five at 4 mm — which
deflates the *mean* for reasons other than the fix; the *worst* column is a
maximum over entries rather than a sum, so that is the like-for-like comparison.
And the run at 0 mm reproduces the previously measured 205.7 µm and 2334.6 µm
exactly, which is what says the sweep is measuring what it claims to.

## 5. What actually helps

<div align="center">
<img src="../figures/10_control_study.png" width="900"/>
</div>

Measured over the **busiest 12 s of the path** — the stretch with the most needle
entries, found from the path rather than fixed, which lands inside the **O**.
Running the study on the opening dots would measure the easy part of the
drawing.

At $\omega_n = 20$ rad/s, full non-linear plant:

| Structure | Settled, marking | Worst settled | Worst, marking | Peak torque |
|---|---|---|---|---|
| PID only | 430.3 µm | 15 010 µm | 30 677 µm | 1.79 N·m |
| PID + gravity | 51.0 µm | 453.7 µm | 622.5 µm | 1.51 N·m |
| PID + gravity + velocity | **40.6 µm** | **320.6 µm** | 863.6 µm | 1.56 N·m |
| Computed torque | 43.7 µm | 339.1 µm | 1185 µm | 1.52 N·m |

Bandwidth sweep, PID + gravity:

| $\omega_n$ | Settled, marking | Worst settled | Peak torque | |
|---|---|---|---|---|
| 10 rad/s | 284.2 µm | 1252 µm | 1.44 N·m | |
| 20 rad/s | 51.0 µm | 453.7 µm | 1.51 N·m | |
| 40 rad/s | **16.8 µm** | 244.1 µm | 1.75 N·m | |
| 80 rad/s | 27 937 µm | 53 486 µm | 20.0 N·m | **torque saturated** |
| 120 rad/s | — | — | — | **diverges** |

### What holds, and what the fix changed

1. **Gravity feedforward is free and is the single biggest win** (758.5 →
   61.0 µm). There has never been a reason not to have it.
2. **The stability cliff is where it has always been.** Between 40 and 80 rad/s,
   set by the 200 Hz `controller_manager` rate, not by the gains. Raising it
   needs a faster controller, not different tuning.
3. **Fixing the trajectory reversed the ranking of the control structures.**
   This is worth stating plainly, because it was measured three times and gave
   three answers.

   When the study ran on the stand-in artwork, computed torque tied with
   velocity feedforward, and the reason given was that the reference
   acceleration comes from finite differences on a path with discontinuous
   velocity, so it injects about as much noise as it corrects.

   When the study moved to the logo, computed torque *won* — the logo has so
   many velocity discontinuities that the acceleration term was correcting more
   than it injected.

   With the slow approach in place, computed torque comes **last** on settled
   mean (90.9 µm against 61.0 for gravity alone), while still giving the best
   worst case. The discontinuities it was correcting for are gone, and what is
   left of the term is its own noise.

   None of the three measurements was wrong. Each was measured on a different
   path, and the acceleration feedforward is only ever as good as the
   acceleration it is handed. That is an argument for giving the trajectory a
   trapezoidal profile with bounded acceleration, not against computed torque.

## 6. Recommended configuration

$\omega_n = 40$ rad/s, PID by pole placement on the effective inertia, with
gravity and velocity feedforward, over a 4 mm slow approach. Same hard window:

| Configuration | Settled, marking | Worst, marking | Peak torque |
|---|---|---|---|
| $\omega_n = 20$, PID + g + v | 40.6 µm | 863.6 µm | 1.56 N·m |
| $\omega_n = 40$, PID + g | 16.8 µm | 288.7 µm | 1.75 N·m |
| $\omega_n = 40$, PID + g + v | **15.2 µm** | **201.4 µm** | 1.81 N·m |

**The whole drawing is inside the line width, worst case included.** 201.4 µm
against 300 µm, with a settled mean of 15.2 µm — a factor of 20 below it — at 9%
of the 20 N·m torque limit.

The worst marking error and the worst *settled* error are now the same number.
That is the clearest statement that the entry transient is gone: the worst
moment of the drawing is no longer a needle entry.

### How this number moved

| Measured on | Settled | Worst | What it was really saying |
|---|---|---|---|
| Stand-in artwork, continuous marking | 22.5 µm | 272 µm | The steady state, on the easy part |
| Logo, busiest window, fast plunge | 205.7 µm | 2335 µm | What the drawing actually asked for |
| Logo, 4 mm approach, 0.6 mm path | 19.3 µm | 178.9 µm | Inside the line, on a coarse reference |
| Logo, 4 mm approach, 0.15 mm path | **15.2 µm** | **201.4 µm** | The same, on a reference worth differentiating |

The middle row is the one worth keeping in view. The first row was not wrong; it
was measured on a stretch of drawing with no needle lifts, and the logo has 98.
Finding that out cost nothing but measuring the right thing.
