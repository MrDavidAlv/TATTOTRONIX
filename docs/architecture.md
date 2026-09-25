# Architecture

How this repository is divided, which way dependencies are allowed to point,
and why. The layering here is checked by `tests/test_architecture.py`, so this
document and the tree cannot drift apart without the build saying so.

---

## The one division that matters

Almost everything here falls on one side or the other of a single line:

| | **Design time** | **Run time** |
|---|---|---|
| Lives in | `docs/scripts/` | `src/` |
| Runs | Before the robot does, on a workstation | On the robot, under ROS 2 |
| Produces | Numbers, figures, `.npz` trajectories | Motion |
| Language | Plain Python, NumPy, matplotlib | `rclpy`, `ros2_control`, launch |
| Fails by | Printing a wrong number | Moving a real needle |

`docs/scripts/` is not documentation with code in it. It is the design
toolchain — forward kinematics, the task Jacobian, damped least-squares inverse
kinematics, recursive Newton–Euler dynamics, the contour tracer and fill, the
control studies — and it is published for one reason: every number in the
documentation has to be re-derivable by whoever reads it. A figure nobody can
regenerate is a claim nobody can check.

**The rule is that the arrow never reverses.** `src/` must not import
`docs/scripts/`. The design toolchain may read the URDF that `src/` owns; the
runtime may not depend on a matplotlib-era script to move a joint. Today that
holds: nothing under `src/` imports it, and the trajectories arrive as data
files instead.

This is why the analysis code is not a ROS package, and why `colcon test` does
not reach it. It has its own suite, `pytest tests/`, and both run in CI.

```
  artwork ──▶ docs/scripts/rospath.py    ──▶ contours, fill
                        │
              kinematics.py (IK)         ──▶ joint angles
                        │
              export_trajectory.py       ──▶ config/trajectories/*.npz
                        │
     ═══════════════════╪════════════ design time above, run time below ═══
                        ▼
              tattotronix_control/draw.py ──▶ FollowJointTrajectory ──▶ arm
```

---

## Layers inside `src/`

Six packages, each allowed to depend only on a strictly lower layer. Same-level
dependencies are rejected as well: two packages at one level that need each
other are one package that has been split for no reason.

| Layer | Package | Build type | Owns | Must not own |
|:---:|---|---|---|---|
| 0 | `tattotronix_description` | `ament_cmake` | The robot itself: xacro, meshes, joint limits, `ros2_control` and Gazebo tags, all behind arguments so one file serves RViz, mock hardware and simulation | Anything that assumes a simulator, a controller or a task |
| 1 | `tattotronix_control` | `ament_python` | The controller set and the drawing application: `tattotronix_controllers.yaml`, the exported trajectories, `draw.py` | Geometry. It reads the description's |
| 1 | `tattotronix_moveit_config` | `ament_python` | Planning: SRDF, kinematics and planner configuration, the `move_group` launch | Controller gains, which belong to layer 1's other half |
| 2 | `tattotronix_gazebo` | `ament_python` | The studio world, and the launch files that assemble simulator, description, spawn, clock bridge and controllers | Anything that would also be true on hardware |
| 2 | `tattotronix_hardware` | `ament_cmake` | The real arm: the `ros2_control` driver for its hobby servos through a PCA9685, and the launch file that assembles description, driver and controllers on a Raspberry Pi | Controller configuration, which it takes from layer 1 unchanged |
| 3 | `tattotronix_bringup` | `ament_python` | One entry point, `draw.launch.py backend:=gazebo\|arm`, that includes the backend's own launch file and hands it the arguments | Any decision a backend makes: what the real arm needs stays in layer 2 |

```
             tattotronix_bringup              layer 3   one entry point
           ╱            ╲
  tattotronix_gazebo   tattotronix_hardware   layer 2   assembles: simulated, real
           ╱            ╲
tattotronix_control   tattotronix_moveit_config   layer 1   commands
           ╲            ╱
        tattotronix_description       layer 0   describes
```

Layer 0 depends on nothing in this repository. That is the property worth
protecting: the robot's geometry can be read by a tool that knows nothing about
Gazebo, `ros2_control` or this project's task, which is exactly what
`docs/scripts/kinematics.py` does.

### Why the controllers are backend agnostic

`tattotronix_control` names no simulator. The same YAML is loaded whether the
hardware interface is `gz_ros2_control`, `mock_components` or the real servos'
driver in `tattotronix_hardware`, which lays only what the hardware has to
change over it: no simulator clock, and a 100 Hz controller manager. If the
simulation and the hardware could carry different controller configurations,
they would, and the difference would be discovered on the hardware.

---

## Where MoveIt fits, and where it does not

MoveIt is not a replacement for the inverse kinematics this project already has.
The reason is the arm itself.

**This is a 5-DOF arm.** A full pose in space needs six numbers, so five joints
cannot reach an arbitrary pose — the general 6-D inverse kinematics problem is
over-determined here, and a full-pose goal generally has no exact solution for
any solver to find.

The project's own solver sidesteps this on purpose. It builds a **5×5 task
Jacobian**: three rows for the tip position and two for the direction of the pen
axis, with the spin about that axis projected out because the needle is a body
of revolution and rotating it about its own axis changes nothing. Five
constraints, five joints, square system. That is the right formulation for this
task, and MoveIt has no reason to be involved in it.

So the split is by motion type. **Only part of it is in place.** Today the
drawing application sends one precomputed trajectory, travel moves included, and
MoveIt is used on its own: to check collisions and to plan on request. Handing it
the travel moves is the intended split, not the current one.

| Motion | Planned by, as intended | Why |
|---|---|---|
| Drawing a stroke | This project's 5×5 task IK, offline | The path is given in Cartesian space to a fraction of a needle width. It is a tracking problem, not a planning one |
| Lifting, travelling between strokes, approach and retract | MoveIt | Free space, no Cartesian requirement, and the one place where the arm can collide with the panel, the table or itself |
| Checking that any of it is collision free | MoveIt's planning scene | The project had no collision checking before it. This is the capability it bought, and the first thing it found was a defect in the collision meshes |

The honest summary is that MoveIt is here for **collision checking
and free-space motion**, not for inverse kinematics. Configured with
`position_only_ik`, a 5-DOF group is well posed for MoveIt's solver — three
constraints against five joints, with two left redundant — and that is enough
for travel moves, which only need the tip to arrive somewhere with the pen
clear of the work.

---

## What the review found

Recorded rather than quietly fixed, because two of the three are judgement calls
that a reader deserves to see argued.

**1. `tattotronix_control` holds two concerns.** The controller configuration is
hardware-facing; `draw.py` and the trajectories are an application. In a stack
with more than one task they would separate, into `_control` and a task or
bringup package. With exactly one task, splitting now would produce a package
containing one node and a launch file, and an extra manifest to keep in step.
It stays merged, and this paragraph is the reason, so the next person does not
have to guess whether it was considered.

**2. Each backend had its own entry point.** The drawing started from
`tattotronix_gazebo/draw.launch.py` in simulation and from
`tattotronix_hardware/arm.launch.py draw:=true` on the arm. Both are thin — the
node, the controllers and the trajectories are shared — but two commands for one
task is one too many. `tattotronix_bringup` now gives it one:
`draw.launch.py backend:=gazebo|arm`. It is deliberately a dispatcher and
nothing more. The real arm has not run yet, and what its launch really needs
will be learnt when it does; that knowledge belongs in `tattotronix_hardware`,
so the bringup only includes the backend's own file and passes the arguments
through, and does not have to change when the hardware launch does. The two
backend entry points stay, for anyone working on one backend alone.

**3. The dependency graph is already acyclic and correctly layered.** It was
checked rather than assumed. `tests/test_architecture.py` now fails the build
if that stops being true, which is the only way it stays true.

---

## Naming

| Pattern | Applies to |
|---|---|
| `tattotronix_<role>` | Every ROS package. `role` is the standard ROS one: `description`, `control`, `gazebo`, `hardware`, `moveit_config`, and `bringup` if it ever exists |
| `joint_1` … `joint_5` | Joints, base outwards |
| `<part>_link` | Links, except `tool0`, which is the ROS convention for the tool flange |
| `tattoo_tcp` | The tool centre point. The frame every Cartesian number in this repository is expressed in |

---

## See also

- **[CONTRIBUTING.md](../CONTRIBUTING.md)** — the cycle a change goes through
- **[docs/mathematical-model/](mathematical-model/)** — what the design toolchain computes, and how
- **[README.md](../README.md#package-layout)** — the same layout, from a user's point of view
