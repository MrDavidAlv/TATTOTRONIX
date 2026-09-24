# Mass Properties

Every mass and inertia the dynamics, the gains and the planner use comes from
here. Until this model existed they came from a bounding box and a guess, and the
guess was wrong by a factor of 8.4.

<div align="center">
<img src="../figures/18_mass_model.png" width="900"/>
</div>

---

## 1. What was wrong

Each link used to be the solid box that fits its bounding box, at a mass written
by hand. The meshes are closed surfaces, so their volumes can be integrated, and
dividing those hand-written masses by them gives the density they implied:
**9.77 g/cm³** across the arm, and up to
14.02 on one link. Steel is 7.85. Nothing on a printed arm is denser than
steel.

## 2. Volume, from the mesh

A closed triangle mesh has an exact volume, centroid and inertia tensor. Each
triangle forms a tetrahedron with the origin; signed volumes count the interior
once and cancel everything outside, and the same decomposition carries the
second moments. That is the divergence theorem written out, in
`docs/scripts/mass_properties.py`, checked against a cube and a sphere before it
is trusted with a robot — including the parallel-axis shift, which is the error
that still produces a plausible tensor.

**Whether the volume means anything** is decided geometrically. The integral of
the outward normal over a closed surface is exactly zero, so the residual
$|\oint \mathbf{n}\,dA| / A$ measures how open a surface is. The worst link closes to
3.2e-03 of its area. Matching edges by shared vertices calls every mesh
open, three to five percent of edges unpaired, but that is duplicated vertices and
T-junctions — an untidy index, not a hole.

## 3. The arm as built

The arm was printed, and it carried **five large servos and two SG90s**. The
shoulder has two large ones, the joint with the most gravity load. One SG90
turns `joint_5`; the other is continuous-rotation, sits in the tool mount's clamp
— 12.0 mm between its arms, an SG90's body width — and drives the tool, not a
joint.

So each link is its printed shell plus the servos mounted in it. A servo that
turns a joint sits on the link before that joint, at the joint's origin, which
the URDF gives exactly; the tool servo sits on the measured tool axis. Shell and
servos are combined properly — masses add, centres of mass average by weight,
and both inertias are carried to the combined centre.

| Link | Volume | Box mass | Implied density | Shell | Servos | As built | Servos in it |
|---|---|---|---|---|---|---|---|
| `base_link` | 117.5 cm³ | 1500 g | 12.76 g/cm³ | 51.0 g | 55 g | **106.0 g** | MG996R joint_1 |
| `shoulder_link` | 110.7 cm³ | 900 g | 8.13 g/cm³ | 48.0 g | 110 g | **158.0 g** | MG996R joint_2, MG996R joint_2 |
| `upper_arm_link` | 64.7 cm³ | 600 g | 9.28 g/cm³ | 28.1 g | 55 g | **83.1 g** | MG996R joint_3 |
| `forearm_link` | 64.1 cm³ | 450 g | 7.02 g/cm³ | 27.8 g | 55 g | **82.8 g** | MG996R joint_4 |
| `wrist_link` | 36.5 cm³ | 350 g | 9.60 g/cm³ | 15.8 g | 9 g | **24.8 g** | SG90 joint_5 |
| `tool_mount_link` | 10.7 cm³ | 150 g | 14.02 g/cm³ | 4.6 g | 9 g | **13.6 g** | SG90 driving the tool |

**0.468 kg** as built against **3.950 kg** in the box model, and the servos
are **62.6%** of it. The mass is concentrated at the joints, not spread along
the links, which is why no uniform density — plastic or aluminium — describes
this arm.

## 4. What is declared, not measured

The volumes are measurements. Three things are not, because the arm they describe
was lost and cannot be weighed, and each is one named constant:

| Declaration | Value | Why this value |
|---|---|---|
| Print density | 0.35 of solid PLA, 434 kg/m³ | A 20% infill part is not 20% of solid; perimeters and top and bottom layers dominate at this size |
| Servo masses | MG996R 55 g, SG90 9 g | Catalogue figures; the models were not confirmed |
| Tool servo position along the bracket | the bracket's centroid | It fills the space between the clamp arms |

The last was measured rather than trusted: moving that servo 5 mm either way
changes the effective inertia of every joint by 1 to 3.4%, and a test holds it
under 5%. Weighing one printed link and one servo, once the arm is rebuilt,
replaces the first two.

## 5. What it changed

- The gains fell by roughly the same factor as the inertia, because every gain
  is pole placement on it. See [parameters](./parameters.md#control).
- The recommended configuration still draws inside the line; lower bandwidths
  got worse, which is consistent with the wrist now carrying more coupling
  relative to its own inertia, though that has not been isolated. See
  [control](./control.md#5-what-actually-helps).
- The planner's acceleration limits cost far less torque. See
  [MoveIt](../moveit.md).
- **Gazebo did not change.** Drawing the logo headless on each model gives the
  same worst joint error to four decimals: the position interface drives each
  joint to its setpoint whatever its inertia. The mass model matters to the
  control design and to hardware, not to the running simulation.

The description keeps both: `mass_model:=printed` is the default and
`mass_model:=box` the comparison.

## 6. What the servos have to supply

<div align="center">
<img src="../figures/25_servo_torque.png" width="780"/>
<br/>
<sub>From the <a href="../notebooks/02_dynamics.ipynb">dynamics notebook</a>: the
gravity and velocity torque each joint needs while drawing the logo, against the
catalogue stall torque of the servos that joint carried.</sub>
</div>

The torque that holds the arm against gravity and carries it along the path,
$G(q) + C(q,\dot q)\dot q$, is the floor of what the servos must give, and it is
set by the masses alone. The shoulder, which carries two MG996R, needs about a
sixth of their combined stall torque; every other joint needs a tenth of its own
or less. Two things keep this from being a margin. The feedback that corrects
errors adds torque on top, and the reference changes velocity instantly at every
corner of the path, which a real controller has to spread out. And a hobby servo
sustains only a fraction of its stall torque, a figure its catalogue does not
state. Sizing the actuators properly is the next piece of work.
