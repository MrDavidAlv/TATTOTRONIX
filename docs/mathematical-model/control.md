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
| Zero pose | 3.0 × 10⁻¹¹ N·m |
| `q = [0.3, −0.5, 0.7, 0.2, −0.4]` | 6.3 × 10⁻¹¹ N·m |
| `q = [−0.8, 1.0, −0.6, 0.9, 1.1]` | 2.0 × 10⁻¹¹ N·m |

Two independent routes — a rigid body recursion against the derivative of a
scalar — agreeing to machine precision.

### A result that looked like a bug and was not

<div align="center">
<img src="../figures/07_gravity.png" width="900"/>
</div>

At the zero pose `joint_2` and `joint_3` carry **the same** gravity torque,
−0.0905 N·m, to the fifth decimal, even though `joint_2` carries more mass above
it.

The reason is geometric. Both axes sit at the same $x$ (28.66 mm), so
everything beyond `joint_3` has the same lever arm about both, and cancels out
of the difference. The only load `joint_2` carries that `joint_3` does not is
the upper arm itself, and its centre of mass sits within a few micrometres of
that line: the printed shell is nearly symmetric about it, and the servo it
carries is on `joint_3`'s axis. So the two torques differ by the upper arm's own
moment, a few micronewton-metres, and a test checks that difference against a
hand computation of $-g\,m\,x$.

Under the earlier box approximation the centre of mass sat *exactly* on the
line and the two torques were identical, which is why this section used to say
so. The version that holds for any mass model is the difference.

> **What these magnitudes are worth.** The volumes behind them are integrated
> from the meshes and are measurements. Two things are declared rather than
> measured, because the arm they describe cannot be weighed: how dense a 20%
> infill print comes out, taken as 0.35 of solid PLA, and the servo figures,
> taken from the catalogue. The *structure* of the model is verified to machine
> precision; the *magnitudes* are as good as those two declarations, and
> [the mass model](./parameters.md#dynamics) lists both.

## 2. Control structure

Independent joint control: one PID per axis, on joint torque, with
feedforward terms added on top:

$$\tau = K_p e + K_i \textstyle\int e \, dt + K_d \dot e + \text{(feedforward)}$$

**Where this loop lives matters, so it is stated plainly.** It is the loop each
joint's servo has to close on the real arm, and this page studies it against the
full non-linear dynamics in `docs/scripts`. It is *not* the loop the running
simulation executes. There, `joint_trajectory_controller` sends joint
*positions*, and Gazebo's plugin drives each joint to its setpoint directly — so
directly that swapping the mass model changes the simulated tracking error in
the fourth decimal. The running system shows the chain works end to end; this
study shows what the actuators must achieve for the drawing to be right.

The studied loop runs at **1 kHz**, the rate `controller_manager` is configured
for in `tattotronix_controllers.yaml`, on the assumption that the servo
interface on hardware sustains the same. The plant is integrated at 1 kHz — 4 kHz
in the rate study below, where the two have to be told apart.

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
differ by a factor of 2.2 on `joint_2` and 3.7 on
`joint_3`, and by 2.5 on `joint_5`, so using the diagonal detunes those axes
by those factors.

| Axis | $1/(M^{-1})_{ii}$ | $\mathrm{diag}(M)$ |
|---|---|---|
| `joint_1` | 0.00264 | 0.00268 |
| `joint_2` | 0.00271 | 0.00584 |
| `joint_3` | 0.000499 | 0.00185 |
| `joint_4` | 0.00000766 | 0.00000954 |
| `joint_5` | 0.0000242 | 0.0000598 |

What matters is not the values but that **every gain traces back to a measured
inertia and a single bandwidth decision**, rather than to a knob someone turned
until it stopped oscillating. That is what made changing the mass model cheap:
when the masses stopped being bounding-box guesses and became the built arm's
- printed shells integrated from the meshes, plus the servos mounted in them -
the gains recomputed themselves, and nothing had to be retuned by hand.

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
raise $\omega_n$, which is bounded by the loop rate ([section 7](#7-the-rate-is-the-ceiling))
and by torque, or feed the reference forward, so the loop no longer has to
generate the velocity.

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
| Marking, settled | **118.2 µm** | 812 µm |
| Marking, all | 121.1 µm | 812 µm |
| Travel | 1782 µm | 3732 µm |

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
needle twice; the logo lifts it **117 times**, because every fill pass that meets
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
| 0.0 mm | 3221 µm | 284.9 µm | 0.27 N·m | 484 s |
| 0.5 mm | 2251 µm | 169.7 µm | 0.26 N·m | 495 s |
| 1.0 mm | 1555 µm | 128.0 µm | 0.27 N·m | 504 s |
| 2.0 mm | 1042 µm | 65.9 µm | 0.27 N·m | 522 s |
| **4.0 mm** | **347 µm** | 26.9 µm | 0.27 N·m | 557 s |

Every step of approach cuts the worst entry, and 4 mm takes it from 3221 to
347 µm for 15% more cycle time and no measurable torque. At this sweep's
bandwidth, 40 rad/s, that is **still outside the 0.3 mm line**. It was inside
when the model carried the box masses; the lighter wrist of the arm as built
gives some of it back. The approach removes most of the entry transient, and
the bandwidth in section 6 removes the rest.

Two honesty notes on that table. The window is 12 s of path, and slowing the
path means fewer entries fall inside it — 8 at 0 mm, 6 at 4 mm — which
deflates the *mean* for reasons other than the fix; the *worst* column is a
maximum over entries rather than a sum, so that is the like-for-like comparison.
And every run starts at the same needle entry, so the rows differ in the
approach and in nothing else.

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
| PID only | 900.8 µm | 25 139 µm | 34 880 µm | 0.28 N·m |
| PID + gravity | 158.5 µm | 1216 µm | 1871 µm | 0.24 N·m |
| PID + gravity + velocity | 167.6 µm | 1164 µm | 1932 µm | 0.24 N·m |
| Computed torque | **143.2 µm** | **803.9 µm** | **1868 µm** | 0.24 N·m |

Bandwidth sweep, PID + gravity:

| $\omega_n$ | Settled, marking | Worst settled | Peak torque | |
|---|---|---|---|---|
| 10 rad/s | 987.5 µm | 3505 µm | 0.23 N·m | |
| 20 rad/s | 158.5 µm | 1216 µm | 0.24 N·m | |
| 40 rad/s | 32.1 µm | 276.1 µm | 0.28 N·m | |
| 80 rad/s | 9.6 µm | 121.7 µm | 0.31 N·m | |
| 160 rad/s | 6.5 µm | 51.1 µm | 0.35 N·m | |
| 240 rad/s | **6.2 µm** | **34.2 µm** | 0.41 N·m | |

There is no cliff in that column any more. There used to be one between 40 and
80 rad/s, and it was never about the gains — see the next section.

### What holds, and what the fix changed

1. **Gravity feedforward is free and is the single biggest win**: 900.8 →
   158.5 µm settled, and 34 880 → 1871 µm at worst. There has never
   been a reason not to have it.
2. **The stability cliff moved with the loop rate, and only with it.** At the
   200 Hz this study first ran at, it sat between 40 and 80 rad/s. At 1 kHz the
   sweep above has none up to 240 rad/s. The gains did not change; the rate
   did. Section 7 measures it.
3. **The ranking of the control structures is not a property of the
   structures.** It has been measured four times and given four answers.

   On the stand-in artwork computed torque tied with velocity feedforward. On
   the logo, with the fast plunge, computed torque won. With the slow approach,
   on the box masses, velocity feedforward won the settled mean and computed
   torque had the worst worst case.

   On the arm as built the order turns again. Computed torque now has the best
   settled mean (143.2 µm) and velocity feedforward the worst
   (167.6 µm), with gravity alone between (158.5 µm); on the worst
   case the three are within 3% of each other. What changed is the masses,
   and with them how much of each joint's load comes from the others: relative
   to its own effective inertia, the coupling the wrist receives rose, most at
   `joint_4`. A structure that models the
   coupling, which is what computed torque is, gains from that; one that only
   feeds each joint its own velocity does not. That is consistent with the
   numbers, and has not been isolated experimentally.

   The same effect is why every structure at this bandwidth tracks worse than
   it did on the box masses. Pole placement normalises each joint by its own
   inertia; split joint by joint, the three large joints track as they did
   before, and the error that grows is the wrist's.

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
| $\omega_n = 40$, PID + g + v | 26.6 µm | 338.0 µm | 0.28 N·m |
| $\omega_n = 160$, PID + g | 6.5 µm | 51.1 µm | 0.35 N·m |
| $\omega_n = 160$, PID + g + v | **6.5 µm** | **46.8 µm** | 0.35 N·m |

**The whole drawing sits inside the line, worst case included**, at about a
sixth of its width: 46.8 µm against 300 µm, with a settled mean of 6.5 µm, at
2% of the 20 N·m torque limit - a limit the description declares as a
placeholder, far above the roughly 1 N·m an MG996R is rated at, so this share
says the loop is not torque-bound in the model, not that the real servos would
have margin. On the box masses the worst case was
somewhat smaller; the arm as built costs a little accuracy at this bandwidth
and almost nothing in settled mean, because 160 rad/s is enough to
overpower the extra coupling that dominates at 20.

240 rad/s is better again on the worst case — 34.2 µm against 51.1 at 160 with
gravity feedforward alone — but it costs more torque and sits closer to the
boundary in section 7. 160 leaves margin on both.

The worst marking error and the worst *settled* error are now the same number.
That is the clearest statement that the entry transient is gone: the worst
moment of the drawing is no longer a needle entry.

### How this number moved

| Measured on | Settled | Worst | What it was really saying |
|---|---|---|---|
| Stand-in artwork, continuous marking | 22.5 µm | 272 µm | The steady state, on the easy part |
| Logo, busiest window, fast plunge | 284.9 µm | 3221 µm | What the drawing actually asked for |
| Logo, 4 mm approach, 0.6 mm path | 53.3 µm | 456.2 µm | The approach, on a coarse reference |
| Logo, 4 mm approach, 0.15 mm path | 26.6 µm | 338.0 µm | The same, on a reference worth differentiating |
| The same, at 1 kHz and 160 rad/s | **6.5 µm** | **46.8 µm** | What the 200 Hz rate had been hiding |

Rows two to five are recomputed on the arm as built; the first is the only
measurement of the stand-in artwork, which the repository no longer carries.

The middle row is the one worth keeping in view. The first row was not wrong; it
was measured on a stretch of drawing with no needle lifts, and the logo has 117.
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
| 200 Hz | 322.0 µm | **saturated** | **diverges** |
| 500 Hz | 359.1 µm | 145.4 µm | **saturated** |
| 1000 Hz | 364.8 µm | 143.9 µm | **77.4 µm** |

Two things fall out of that table.

**The ceiling scales with the rate.** Each row's usable bandwidth roughly doubles
as the rate doubles. Every working point has $\omega_n$ at or below about a
quarter of the sample rate, and every failing one is above a third; the boundary
lies between, and this sweep does not resolve it more finely.

**Raising the rate alone buys nothing.** The $\omega_n$ = 40 column does not
improve across a fivefold change in rate — 322 to 365 µm, if anything a
little worse. The rate is not a source of accuracy, it is permission to ask
for more bandwidth, and the bandwidth is what delivers.

**And the failure is torque, not arithmetic.** At 200 Hz and 80 rad/s the loop
does not produce non-finite numbers: it commands the full 20 N·m and sits there
at 58 mm of error. A discrete loop asked for a bandwidth close to its own rate
overshoots between samples and then demands whatever torque it takes to correct,
which is a much more recognisable failure on real hardware than a NaN.

### What this costs outside simulation

In simulation, nothing: `update_rate: 1000` in
`tattotronix_controllers.yaml` and the ceiling moves.

On hardware it is a requirement on the servo interface rather than a preference.
A bus that cannot sustain 1 kHz puts the ceiling back where it was, and with it
the 322 µm worst case. That makes the interface rate a specification for the
hardware this model is meant to describe, which is the kind of number worth
knowing before buying anything.
