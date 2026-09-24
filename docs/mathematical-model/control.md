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

The loop closes at **1 kHz**, the `controller_manager` rate in
`tattotronix_controllers.yaml`. The plant in simulation is the full non-linear
dynamics, integrated at 1 kHz — 4 kHz in the rate study below, where the two
have to be told apart.

That rate is not a free choice: it is what sets the achievable bandwidth, and it
used to be 200 Hz. See [the ceiling](#7-the-rate-is-the-ceiling).

### Tuning

Each axis is a double integrator, $J\ddot q = \tau$, so a PID closes it as

$$J s^3 + K_d s^2 + K_p s + K_i$$

Placing **all three poles at $-\omega_n$** gives $(s + \omega_n)^3$, and therefore

$$K_p = 3 J \omega_n^2, \qquad K_d = 3 J \omega_n, \qquad K_i = J \omega_n^3$$

which has no overshoot by construction.

**Two details that are not cosmetic.**

An earlier version used the second order PD rule, $K_p = J\omega_n^2$ and
$K_d = 2\zeta\omega_n J$, with an integral term bolted on as a fraction of $K_p$.
That put the integral time at 0.33 s against a loop time constant of 0.05 s, and
the step response overshot by 30 to 75 percent.

And $J$ is the **effective** inertia, $1/(M^{-1})_{ii}$, not the diagonal of $M$.
The two are not the same: the diagonal says how much inertia the axis carries
*if every other axis is frozen*, and nothing freezes them. At the zero pose they
differ by a factor of 2.2 on `joint_2` and 2.8 on `joint_3`, so using the
diagonal detunes those two axes by that factor.

| Axis | $1/(M^{-1})_{ii}$ | $\mathrm{diag}(M)$ |
|---|---|---|
| `joint_1` | 0.01561 | 0.01562 |
| `joint_2` | 0.01552 | 0.03442 |
| `joint_3` | 0.00366 | 0.01016 |
| `joint_4` | 0.00019 | 0.00020 |
| `joint_5` | 0.00007 | 0.00010 |

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
| Marking, settled | **94.7 µm** | 711 µm |
| Marking, all | 99.4 µm | 711 µm |
| Travel | 1303 µm | 3541 µm |

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
| PID only | 442.0 µm | 16 200 µm | 30 037 µm | 1.78 N·m |
| PID + gravity | 48.1 µm | 450.5 µm | 598.8 µm | 1.51 N·m |
| PID + gravity + velocity | **39.6 µm** | **319.2 µm** | 820.4 µm | 1.55 N·m |
| Computed torque | 43.4 µm | 339.8 µm | 1136 µm | 1.51 N·m |

Bandwidth sweep, PID + gravity:

| $\omega_n$ | Settled, marking | Worst settled | Peak torque | |
|---|---|---|---|---|
| 10 rad/s | 289.6 µm | 1271 µm | 1.44 N·m | |
| 20 rad/s | 48.1 µm | 450.5 µm | 1.51 N·m | |
| 40 rad/s | 17.0 µm | 246.5 µm | 1.72 N·m | |
| 80 rad/s | 7.8 µm | 114.3 µm | 1.91 N·m | |
| 160 rad/s | **6.2 µm** | 39.3 µm | 2.40 N·m | |
| 240 rad/s | 6.1 µm | **28.4 µm** | 2.68 N·m | |

There is no cliff in that column any more. There used to be one between 40 and
80 rad/s, and it was never about the gains — see the next section.

### What holds, and what the fix changed

1. **Gravity feedforward is free and is the single biggest win**: 442.0 →
   48.1 µm settled, and 30 037 → 598.8 µm at worst. There has never been a
   reason not to have it.
2. **The stability cliff moved with the loop rate, and only with it.** At the
   200 Hz this study first ran at, it sat between 40 and 80 rad/s. At 1 kHz the
   sweep above has none up to 240 rad/s. The gains did not change; the rate
   did. Section 7 measures it.
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

   With the slow approach in place, the table above splits the two measures.
   On settled mean velocity feedforward is best (39.6 µm), computed torque sits
   in between (43.4 µm) and gravity alone is last (48.1 µm). On the worst case
   the order inverts: gravity alone 598.8 µm, velocity feedforward 820.4 µm,
   computed torque 1136 µm. Computed torque is the structure most exposed at
   the path's worst moments, which is consistent with the explanation given
   the first time - its acceleration comes from finite differences, and those
   are noisiest exactly where the reference changes abruptly. That explanation
   fits every measurement so far; it has not been isolated experimentally.

   None of the three measurements was wrong. Each was measured on a different
   path, and the acceleration feedforward is only ever as good as the
   acceleration it is handed. That is an argument for giving the trajectory a
   trapezoidal profile with bounded acceleration, not against computed torque.

## 6. Recommended configuration

$\omega_n = 160$ rad/s, PID by pole placement on the effective inertia, with
gravity and velocity feedforward, over a 4 mm slow approach, at a 1 kHz loop
rate. Same hard window:

| Configuration | Settled, marking | Worst, marking | Peak torque |
|---|---|---|---|
| $\omega_n = 40$, PID + g + v | 15.2 µm | 200.0 µm | 1.77 N·m |
| $\omega_n = 160$, PID + g | 6.2 µm | 40.4 µm | 2.40 N·m |
| $\omega_n = 160$, PID + g + v | **6.3 µm** | **36.3 µm** | 2.37 N·m |

**The whole drawing sits at an eighth of the line width, worst case included.**
36.3 µm against 300 µm, with a settled mean of 6.3 µm, at 12% of the 20 N·m
torque limit.

240 rad/s is slightly better again — 28.4 µm — but it costs more torque and sits
closer to the boundary in section 7. 160 leaves margin on both.

The worst marking error and the worst *settled* error are now the same number.
That is the clearest statement that the entry transient is gone: the worst
moment of the drawing is no longer a needle entry.

### How this number moved

| Measured on | Settled | Worst | What it was really saying |
|---|---|---|---|
| Stand-in artwork, continuous marking | 22.5 µm | 272 µm | The steady state, on the easy part |
| Logo, busiest window, fast plunge | 205.7 µm | 2335 µm | What the drawing actually asked for |
| Logo, 4 mm approach, 0.6 mm path | 19.3 µm | 178.9 µm | Inside the line, on a coarse reference |
| Logo, 4 mm approach, 0.15 mm path | 15.2 µm | 200.0 µm | The same, on a reference worth differentiating |
| The same, at 1 kHz and 160 rad/s | **6.3 µm** | **36.3 µm** | What the 200 Hz rate had been hiding |

The middle row is the one worth keeping in view. The first row was not wrong; it
was measured on a stretch of drawing with no needle lifts, and the logo has 98.
Finding that out cost nothing but measuring the right thing.

---

## 7. The rate is the ceiling

Every bandwidth sweep in this project used to stop at 40 rad/s, and every time
the conclusion was "the gains cannot go higher". They could. What could not go
higher was the rate the loop ran at.

<div align="center">
<img src="../figures/14_rate.png" width="900"/>
</div>

Sweeping the two together, gravity and velocity feedforward, worst marking error:

| Rate | $\omega_n$ = 40 | $\omega_n$ = 80 | $\omega_n$ = 160 |
|---|---|---|---|
| 200 Hz | 220.1 µm | **saturated** | **diverges** |
| 500 Hz | 215.2 µm | 129.5 µm | **saturated** |
| 1000 Hz | 214.2 µm | 127.9 µm | **67.9 µm** |

Two things fall out of that table.

**The ceiling scales with the rate.** Each row's usable bandwidth roughly doubles
as the rate doubles. Every working point has $\omega_n$ at or below about a
quarter of the sample rate, and every failing one is above a third; the boundary
lies between, and this sweep does not resolve it more finely.

**Raising the rate alone buys nothing.** The $\omega_n$ = 40 column barely moves
across a fivefold change in rate — 220 to 214 µm. The rate is not a source of
accuracy, it is permission to ask for more bandwidth, and the bandwidth is what
delivers.

**And the failure is torque, not arithmetic.** At 200 Hz and 80 rad/s the loop
does not produce non-finite numbers: it commands the full 20 N·m and sits there
at 24 mm of error. A discrete loop asked for a bandwidth close to its own rate
overshoots between samples and then demands whatever torque it takes to correct,
which is a much more recognisable failure on real hardware than a NaN.

### What this costs outside simulation

In simulation, nothing: `update_rate: 1000` in
`tattotronix_controllers.yaml` and the ceiling moves.

On hardware it is a requirement on the servo interface rather than a preference.
A bus that cannot sustain 1 kHz puts the ceiling back where it was, and with it
the 200 µm worst case. That makes the interface rate a specification for the
hardware this model is meant to describe, which is the kind of number worth
knowing before buying anything.
