# TATTOTRONIX 5 DOF Tattoo Manipulator

<div align="center">
<img src="docs/figures/simulation.gif" width="70%"/>
<br/>
<sub>The arm drawing the official ROS logo in Gazebo, with the ink shown as an
RViz marker — nothing in a simulator leaves a mark when a tool passes over a
surface. Sped up. Recorded on an earlier revision of the path, whose run took
561 s; the current trajectory takes 557 s at a 6 mm/s marking feed, plus a few
seconds for the arm to settle before it starts.
<a href="docs/figures/simulation.mp4">Full clip (MP4)</a>.</sub>
</div>

</br>

<div align="center" width="70%">

[![CI](https://github.com/MrDavidAlv/TATTOTRONIX/actions/workflows/ci.yml/badge.svg)](https://github.com/MrDavidAlv/TATTOTRONIX/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)](#)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Ignition Fortress](https://img.shields.io/badge/Gazebo-Fortress-orange)](#)
[![ros2_control](https://img.shields.io/badge/ros2__control-Humble-00599C)](#)
[![Colab: Kinematics](https://img.shields.io/badge/Colab-Kinematics-F9AB00?logo=googlecolab&logoColor=white)](https://colab.research.google.com/drive/1Lmc4HzBdJf-z6v3CmRNt6g5lY85Nq4Dd?usp=sharing)
[![Colab: Dynamics](https://img.shields.io/badge/Colab-Dynamics-F9AB00?logo=googlecolab&logoColor=white)](https://colab.research.google.com/drive/16LlfGKtVIHmqNoxojnTrFz47NQBHQEkX?usp=sharing)
[![Colab: Control](https://img.shields.io/badge/Colab-Control-F9AB00?logo=googlecolab&logoColor=white)](https://colab.research.google.com/github/MrDavidAlv/TATTOTRONIX/blob/humble/docs/notebooks/03_control.ipynb)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![GitHub](https://img.shields.io/badge/GitHub-MrDavidAlv-181717?logo=github)](https://github.com/MrDavidAlv/TATTOTRONIX)

</div>

---

## Quick Start

### In a container

The environment this needs is specific enough to be worth pinning — see
[Known Issues](#known-issues) for one way it goes wrong that does not look like
an environment problem at all. The container is also what CI runs, so a green
build and a working checkout are the same claim.

```bash
export HOST_UID=$(id -u) HOST_GID=$(id -g)   # only if your user is not 1000
docker compose build

docker compose run --rm dev                  # a shell, no graphics
docker compose run --rm dev colcon test
docker compose run --rm gui ros2 launch tattotronix_gazebo draw.launch.py

docker compose build ci && docker compose run --rm ci   # exactly what CI checks
```

Run `ci` before pushing, and build it first: it runs the code baked into the
image, so without a build it tests whatever you built last. It is the same image with **nothing mounted over it**,
which is the difference that matters: `dev` mounts the workspace, so a file
missing from the image is still there because the mount put it back. The
document certification passed that way and failed the moment CI ran it without
a mount.

`dev` is headless and is what the analysis scripts use; CI uses `ci`. `gui` is the same
image with the host display handed in, for watching the simulation. They are
separate because a container that needs an X socket fails on a machine with
none, and CI is exactly that machine.

The workspace is mounted, so edits on the host are what runs; `build/` and
`install/` stay in named volumes, because mixing a host build tree with a
container one breaks both. Those volumes persist between runs and are not
rebuilt for you: after pulling a change that adds or alters a package, run
`colcon build --symlink-install` inside `dev` once, or it will not find it.

### On the host

```bash
# 1. ROS 2 Humble on Ubuntu 22.04
sudo apt update && sudo apt install ros-humble-desktop

# 2. Dependencies
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
                 ros-humble-gz-ros2-control ros-humble-ros2-control \
                 ros-humble-ros2-controllers ros-humble-xacro \
                 ros-humble-joint-state-publisher-gui ros-humble-moveit
# or, from the checkout, exactly what the package manifests declare:
#   rosdep install --from-paths src --ignore-src -y

# 3. Build
git clone git@github.com:MrDavidAlv/TATTOTRONIX.git
cd TATTOTRONIX
colcon build --symlink-install
source install/setup.bash

# 4a. Look at the model, drive the joints by hand
ros2 launch tattotronix_description display.launch.py

# 4b. Or run it in Gazebo under ros2_control
ros2 launch tattotronix_gazebo simulation.launch.py
```

---

## Table of Contents

- [Description](#description)
- [Analysis and Model](#analysis-and-model)
- [Notebooks](#notebooks)
- [Kinematics](#kinematics)
- [Package Layout](#package-layout)
- [Architecture](#architecture)
- [Planning with MoveIt](#planning-with-moveit)
- [Description Arguments](#description-arguments)
- [Visualization](#visualization)
- [Simulation](#simulation)
- [Control](#control)
- [Migration Notes](#migration-notes)
- [What Is Real and What Is Placeholder](#what-is-real-and-what-is-placeholder)
- [Known Issues](#known-issues)
- [Usage](#usage)
- [Contributing](#contributing)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)

---

## Description

TATTOTRONIX is a 5 degree of freedom desktop manipulator built to hold a tattoo
pen. Nothing below the tool flange knows that: the arm, its controllers and its
simulation are a general purpose small manipulator, and the tattoo machine is a
separate macro bolted to `tool0`. Fitting a different end effector is a launch
argument, not an edit.

The mechanical design is the project's own CAD, exported as six STL meshes. At
the zero pose the wrist axis stands 242 mm above the bench and the highest point
of the arm is 292 mm. The tool flange reaches 214 mm forward, and the pen adds
another 45 mm, putting the tip at 259 mm.

This branch is the ROS 2 Humble port. The repository started from ROS Industrial
training material on catkin; what carried over is the CAD and the joint
geometry, and everything else is new. See [Migration Notes](#migration-notes).

---

## Analysis and Model

The arm is modelled end to end and every figure below is regenerated by a script
that reads the same URDF `robot_state_publisher` loads, so the model cannot drift
away from the robot. The full write-up lives in
**[docs/mathematical-model/](docs/mathematical-model/)**:

| | |
|---|---|
| **[Kinematics](docs/mathematical-model/kinematics.md)** — the chain, forward kinematics verified against live TF, the 5×5 task Jacobian, and why five axes are exactly enough | **[Toolpath](docs/mathematical-model/toolpath.md)** — artwork to ink mask to contour and fill, Moore neighbour tracing, and what the stand-in artwork was hiding |
| **[Control](docs/mathematical-model/control.md)** — Newton–Euler dynamics, tuning by pole placement, the velocity lag and why the stability cliff was set by the loop rate | **[Parameters](docs/mathematical-model/parameters.md)** — every value with its source, and what the model cannot tell you |
| **[Mass properties](docs/mathematical-model/mass.md)** — volumes integrated from the meshes, and the arm as built: printed shells plus five large servos and two SG90s | |

<div align="center">
<img src="docs/figures/drawing.gif" width="88%"/>
<br/>
<sub>The same path rendered directly from the solved trajectory by
<a href="docs/scripts/render_drawing.py">render_drawing.py</a>: the arm in
elevation, the panel face on. 557 s compressed into 24 —
<a href="docs/figures/drawing.mp4">MP4</a>.</sub>
</div>

<div align="center">
<img src="docs/figures/21_arm_3d.gif" width="70%"/>
<br/>
<sub>And in 3D, from the <a href="docs/notebooks/01_kinematics.ipynb">kinematics
notebook</a>: the arm's real meshes, simplified for animation, replaying the same
trajectory.</sub>
</div>

### Notebooks

The robotics behind all of this, worked through step by step and runnable in the
browser, with no ROS installed:

| Notebook | | |
|---|---|---|
| **[Kinematics](docs/notebooks/01_kinematics.ipynb)** | Rotations and Rodrigues' formula, homogeneous transforms, forward kinematics derived symbolically, the product of exponentials, the space, geometric and task Jacobians, manipulability ellipsoids, damped least-squares inverse kinematics, and the arm drawing in 2D and 3D | [![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/1Lmc4HzBdJf-z6v3CmRNt6g5lY85Nq4Dd?usp=sharing) |
| **[Dynamics](docs/notebooks/02_dynamics.ipynb)** | Inertia tensors and the spatial inertia matrix, the equations of motion, the mass matrix from kinetic energy, Coriolis terms from Christoffel symbols, gravity from potential energy, energy conservation with the arm falling freely, and the torque each servo has to supply | [![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/16LlfGKtVIHmqNoxojnTrFz47NQBHQEkX?usp=sharing) |
| **[Control](docs/notebooks/03_control.ipynb)** | Pole placement derived symbolically, where the overshoot comes from and what feedforward does to it, the following error in closed form, the modes the coupling splits the arm's loop into, the sampled loop and its exact rate ceiling, and the non-linear arm past it | [![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/MrDavidAlv/TATTOTRONIX/blob/humble/docs/notebooks/03_control.ipynb) |

Each formula is implemented where it can be read and then checked against the
model the repository runs, and every notebook is executed on each build, so their
`assert` lines are part of the certification. See
[docs/notebooks](docs/notebooks/).

### Figures

| | |
|---|---|
| <img src="docs/figures/01_chain.png" width="420"/><br/>**The chain.** Forward kinematics against live TF, agreeing to the resolution `tf2_echo` prints. | <img src="docs/figures/02_workspace.png" width="420"/><br/>**Workspace.** Uniform sampling of the joint box. The panel falls inside the envelope. |
| <img src="docs/figures/04_toolpath.png" width="420"/><br/>**Toolpath.** The official ROS logo: 2558 mm marked, 3455 mm travelled, 117 needle entries. | <img src="docs/figures/11_contours.png" width="420"/><br/>**Contour tracing.** Sorting boundary pixels by angle works on a disc and collapses on a letterform. |
| <img src="docs/figures/07_gravity.png" width="420"/><br/>**Gravity torque.** Newton–Euler against the gradient of potential energy, agreeing to 10⁻¹¹ N·m. | <img src="docs/figures/08_step.png" width="420"/><br/>**Step response.** Every gain traces back to a measured inertia and one bandwidth decision. |
| <img src="docs/figures/09_tracking.png" width="420"/><br/>**Tracking.** Settled the tip holds 118.2 µm; the spikes are needle entries and exits. | <img src="docs/figures/05_joint_trajectories.png" width="420"/><br/>**Joint trajectories.** `joint_4` sits at exactly zero, the correct answer for flat work. |
| <img src="docs/figures/10_control_study.png" width="420"/><br/>**Control study.** Over the busiest stretch of the logo, where the needle lifts most. | <img src="docs/figures/12_approach.png" width="420"/><br/>**Needle entry.** Landing the last 4 mm at marking feed took the worst entry from 3.2 mm to 347 µm at 40 rad/s; the recommended bandwidth takes the rest inside the line. |
| <img src="docs/figures/13_resample.png" width="420"/><br/>**Resampling.** A finer path lowers the settled error, because the feedforward differentiates it; on the arm as built the worst case no longer follows. | <img src="docs/figures/14_rate.png" width="420"/><br/>**Controller rate.** The bandwidth ceiling was never the gains — it was the loop rate, 200 Hz at the time. |
| <img src="docs/figures/06_manipulability.png" width="420"/><br/>**Conditioning.** Condition number 21 to 36; nowhere near a singularity. | <img src="docs/figures/03_panel.png" width="420"/><br/>**Panel reachability.** The needle can be put perpendicular over 98.4% of the surface. |
| <img src="docs/figures/19_frames.png" width="420"/><br/>**Frames.** The frame of every joint and of the needle tip, at a pose from the drawing. | <img src="docs/figures/20_manipulability.png" width="420"/><br/>**Manipulability.** Singular values over the drawing, and the tip's ellipsoid at nine points of it. |
| <img src="docs/figures/22_ik_convergence.png" width="420"/><br/>**Inverse kinematics.** Damped least squares written out by hand, matching the repository's solver to 10⁻¹². | <img src="docs/figures/17_moveit_planning.png" width="420"/><br/>**MoveIt.** The planning scene against the corrected collision geometry: no link in collision. |
| <img src="docs/figures/23_mass_matrix.png" width="420"/><br/>**Mass matrix.** Coupling between joints for both mass models, and how much of it reaches the wrist. | <img src="docs/figures/24_free_fall.png" width="420"/><br/>**Energy.** The arm falling freely: kinetic and potential trade places and the total stays flat. |
| <img src="docs/figures/25_servo_torque.png" width="420"/><br/>**Servo torque.** Gravity and velocity torque while drawing, against each joint's servos. | <img src="docs/figures/21_arm_3d.gif" width="420"/><br/>**In 3D.** The arm's real meshes replaying the drawing. |
| <img src="docs/figures/26_loop_structures.png" width="420"/><br/>**Loop structures.** One PID, three places for the reference to enter: the overshoot and the lag each one leaves. | <img src="docs/figures/27_coupled_modes.png" width="420"/><br/>**Coupled modes.** The arm splits the per-joint loops into five modes; none of them is the loop that was designed. |
| <img src="docs/figures/28_rate_ceiling.png" width="420"/><br/>**The rate ceiling.** Stable while ωnT < 2/(3λmax): the rule that sorts all nine runs of the rate study. | <img src="docs/figures/29_past_the_ceiling.png" width="420"/><br/>**Past the ceiling.** The non-linear arm grows at the rate the sampled loop predicts, until the torque limit. |
| <img src="docs/figures/18_mass_model.png" width="420"/><br/>**Mass model.** The box approximation made the arm eight times too heavy; as built, the servos are most of it. | <img src="docs/figures/16_collision_after.png" width="420"/><br/>**Collision geometry.** The shape every collision query reads, after the double transform was removed. |
| <img src="docs/images/gazebo_simulation.png" width="420"/><br/>**Gazebo.** The arm under `ros2_control` in Ignition Fortress. | <img src="docs/images/rviz_display.png" width="420"/><br/>**RViz.** Joint origins and the tool frames. |

### Watching it draw

The whole chain is solved offline and exported to the control package, so the
arm in simulation follows the same trajectory every number here was measured
against:

```bash
ros2 launch tattotronix_gazebo draw.launch.py              # the ROS logo
ros2 launch tattotronix_gazebo draw.launch.py art:=foto    # something else
```

`art` picks one of the trajectories installed by `tattotronix_control`. Give it
a name it does not have and it lists the ones it does.

| `art:=` | Drawing | Method | Points | Marked | At the planned feed |
|---|---|---|---|---|---|
| `ros_logo` *(default)* | The official ROS logo | threshold | 16 656 | 2558 mm | 9 min |
| `hagamos` | A wordmark | threshold | 28 468 | 4229 mm | 15 min |
| `semillero` | A robotics club logo | threshold | 44 066 | 6717 mm | 26 min |
| `foto` | A photograph, as line art | edges | 40 330 | 5320 mm | 38 min |

Inverse kinematics converges at 100% of the path points for all four.

A fifth, `ingeniero`, is not in the repository. It is about 4 MB and fully
reproducible from an image that *is* here, which makes it the one derived
artifact worth regenerating rather than versioning — the other four come from
artwork this repository cannot publish, so their trajectory is the only copy.

```bash
python3 docs/scripts/export_trajectory.py \
    --image docs/images/ingeniero.png --name ingeniero --pitch 0.5
```

The ink appears in RViz on `/ink_trace`. Nothing in Gazebo leaves a mark when a
tool passes over a surface, so without that marker the arm moves for nine
minutes and nothing appears. The marker draws the *commanded* path; how closely
the loop follows it is [measured separately](docs/mathematical-model/control.md),
on the full non-linear plant.

> **`speed` is not a playback rate.** It rescales the motion itself, so
> `speed:=6` asks the arm to mark at 36 mm/s instead of 6. Past roughly 2x the
> arm falls outside `joint_trajectory_controller`'s 0.1 rad trajectory
> tolerance and the goal is aborted part-way through — the arm stops dead and
> the drawing is left unfinished. That tolerance is doing its job.

### Drawing something else

Any raster image works. The mask is built, the toolpath planned, inverse
kinematics solved, and the result written where `draw` will find it:

```bash
python3 docs/scripts/export_trajectory.py --image mylogo.png --name mylogo
ros2 launch tattotronix_gazebo draw.launch.py art:=mylogo
```

Rebuild `tattotronix_control` afterwards so the new file is installed.

**Pick the method to match the artwork.** `--method threshold`, the default,
finds solid regions and suits logos, lettering and flat illustration. The
polarity is decided rather than assumed — ink is whichever side of the threshold
covers less than half the image — so artwork on a black field works without a
flag. `--method edges` traces outlines instead, and is the honest choice for a
photograph: thresholding a continuous-tone image fuses hair and dark clothing
into one blob, and recovering that detail with a local threshold came to 34
hours of marking on a 150 mm portrait when it was tried. Edges give line art in a fraction of it.

**`--pitch` is the fill spacing, and it is not the same as coverage.** The
default 1.2 mm leaves a 0.9 mm gap between passes of a 0.3 mm needle, which is
hatching rather than filling — legible on bold shapes, hollow on anything
finer. A pitch at or below the line width fills solid, at four times the marking
time. Three of the trajectories above use 0.5 mm, which reads as filled without
taking half an hour; the ROS logo keeps the 1.2 mm default, which is what every
figure in the analysis was measured on.

It refuses to export a trajectory whose inverse kinematics did not converge
everywhere, rather than hand the arm a path it cannot follow.

### Reproducing

```bash
docs/scripts/fetch_artwork.sh                  # the official ROS logo, CC BY-NC, not vendored
python3 docs/scripts/rospath.py                # self-checks the contour tracer, then path stats
python3 docs/scripts/mass_properties.py --write-xacro --write-data   # the mass model
python3 docs/scripts/analysis.py               # 2 min, writes docs/data/summary.json
python3 docs/scripts/kinematics_study.py       # 4 min
python3 docs/scripts/control_study.py          # 15 min
python3 docs/scripts/approach_study.py         # 6 min
python3 docs/scripts/resample_study.py         # 3.5 min
python3 docs/scripts/rate_study.py             # 33 min
python3 docs/scripts/make_moveit_config.py     # SRDF and joint limits
docs/scripts/moveit_check.sh                   # asks move_group what docs/moveit.md claims
python3 docs/scripts/figures.py                # writes docs/figures/*.png
python3 docs/scripts/export_trajectory.py      # trajectory for the draw node
python3 docs/scripts/render_drawing.py         # the animation at the top
python3 docs/scripts/check_docs.py             # certifies the documents against the data
python3 docs/notebooks/build.py --export 01_kinematics 02_dynamics 03_control   # the notebooks' figures
```

The studies are independent of each other and can run in parallel; times are
single runs on a 16-core workstation.

`check_docs.py` is the one that matters if you change anything. It fails if a
headline number no longer matches the data file it came from, if any figure in
µm or N·m is absent from the data without a stated reason, if the gains, torque
shares, inertia table, mass table or kinematic figures disagree with their
data, or if a link, image or script path is broken. It cannot see a claim with
no number in it; those have to be re-read against the data by hand.

Beyond ROS 2 Humble these need `numpy`, `scipy`, `matplotlib`, `pillow` and
`cairosvg`. See [Known Issues](#known-issues) about `PATH` and OSS CAD Suite.

---

## Kinematics

Five revolute axes, all limited to +-1.57 rad, plus a fixed tool flange.

| Joint | Parent | Child | Axis | Origin from parent (m) |
|-------|--------|-------|------|------------------------|
| `joint_1` | `base_link` | `shoulder_link` | Z, yaw | `0, 0, 0.05` |
| `joint_2` | `shoulder_link` | `upper_arm_link` | Y, pitch | `0.02866, -0.00364, 0.0431385` |
| `joint_3` | `upper_arm_link` | `forearm_link` | Y, pitch | `0, 0.0251, 0.1201` |
| `joint_4` | `forearm_link` | `wrist_link` | X, roll | `0.0033, -0.02184, 0.02885` |
| `joint_5` | `wrist_link` | `tool_mount_link` | Y, pitch | `0.153, 0.003, -0.002` |
| `tool_mount_to_tool0` | `tool_mount_link` | `tool0` | fixed | `0.029, -0.012064, 0.005489`, rpy `0, pi/2, 0` |

The frame tree, from `check_urdf`:

```
world -> base_link -> shoulder_link -> upper_arm_link -> forearm_link
      -> wrist_link -> tool_mount_link -> tool0 -> tattoo_pen_link -> tattoo_tcp
```

`tool0` is the flange every end effector attaches to, and its +z axis is the
tool axis, so a tool mounts with no rotation of its own. `tattoo_tcp` sits at
the pen tip and is the frame tattoo paths are planned in.

The joint origins came across unchanged from the ROS 1 description, converted
from the CAD centimetre units to metres. The meshes are exported in their own
joint frames, so each visual and collision sits at its link origin and needs
only the 0.01 scale factor.

### The tool mount

`tool_mount_link` is a printed bracket holding a continuous-rotation SG90. Its
two clamp arms face each other across a 12.0 mm gap, which is the body width of
an SG90, and their mid plane at y = -12.06 mm is what every turned feature at
the far end of the bracket is centred on.

The bracket carries two sets of three holes and they are easy to confuse. The
set bored along y, radius 1.50 mm at x = 0 and z = -8, 0 and +8 mm, passes
through the 3 mm back plate and is how the bracket bolts down; it carries
nothing. The set bored along x through the outer face is the one the tool uses:
radius 1.50 mm, on a straight line at 8.00 mm pitch, centres at (-5.203,
+9.603), (-12.064, +5.489) and (-18.925, +1.375) in mm.

`tool0` sits on the middle of that line, at x = 29.0 mm where the outer face
ends, and is rotated so its +z runs along the bracket's +x. Behind that hole the
face opens into a stepped boss of radius 2.50, 3.00 and 4.00 mm, concentric with
it to within 5 um, which is a bearing seat and not a bolt hole.

The servo drives the tool, not the arm: it is one of two SG90s on the built arm,
the other turning `joint_5`. The description records the geometry and actuates
nothing here — the tool hangs off a fixed joint — but the servo's 9 g is in the
mass model, on the tool axis.

---

## Package Layout

```
src/
  tattotronix_description/     URDF/xacro, meshes, generated inertials, RViz config, display launch
  tattotronix_control/         controller YAML, spawners, the draw node and its trajectories
  tattotronix_moveit_config/   SRDF, kinematics and planner config, move_group launch
  tattotronix_gazebo/          Gazebo Sim world, the simulation and draw launches
docs/scripts/                  the design toolchain: models, studies, figures, certification
tools/
  align_collision_meshes.py    puts each collision DAE in its visual's frame
```

| Package | Build type | What it owns |
|---------|-----------|--------------|
| `tattotronix_description` | `ament_cmake` | The robot. Geometry, kinematics, ros2_control and Gazebo tags, all behind arguments so one file serves RViz, mock hardware and simulation |
| `tattotronix_control` | `ament_python` | The controller set, backend agnostic on purpose so simulation and hardware cannot drift apart, and the drawing application with its trajectories |
| `tattotronix_moveit_config` | `ament_python` | Planning: SRDF and joint limits generated from the description, position-only IK, OMPL, the `move_group` launch |
| `tattotronix_gazebo` | `ament_python` | The studio world, and the launch files that assemble simulator, description, spawn, clock bridge and controllers, and run the drawing |

---

## Architecture

**[docs/architecture.md](docs/architecture.md)** is the full record: the layering,
the reasoning behind each boundary, what the review found and chose not to
change, and where MoveIt fits.

The short version is one line — **design time and run time are separate, and the
arrow never reverses.** `docs/scripts/` is the design toolchain: kinematics,
dynamics, the contour tracer, the control studies. It runs on a workstation
before the robot moves and its output is data. `src/` is the runtime. Nothing
under `src/` imports `docs/scripts/`, and a test fails the build if that changes.

Inside `src/`, packages depend strictly downwards:

```
        tattotronix_gazebo            layer 2   assembles
           /            \
tattotronix_control   tattotronix_moveit_config   layer 1   commands
           \            /
        tattotronix_description       layer 0   describes
```

Layer 0 depends on nothing here, which is what lets a tool that knows nothing
about Gazebo or `ros2_control` read the robot's geometry — exactly what
`docs/scripts/kinematics.py` does. `tests/test_architecture.py` reads the
manifests and fails on any edge that points the wrong way.

---

## Planning with MoveIt

**[docs/moveit.md](docs/moveit.md)** is the full account. MoveIt is here for
**collision checking and free-space motion**, not for inverse kinematics: five
joints cannot reach an arbitrary six-number pose, so a full-pose goal generally
has no exact solution, while the drawing is already solved offline by this
project's 5x5 task Jacobian. Today it runs alongside the drawing rather than
inside it - travel between strokes is still part of the offline trajectory -
and handing it those moves is the intended next step.

```bash
ros2 launch tattotronix_gazebo simulation.launch.py
ros2 launch tattotronix_moveit_config move_group.launch.py
```

The SRDF and the joint limits are **generated**, not written:
`docs/scripts/make_moveit_config.py` reads the URDF and the measured dynamics,
and a test regenerates and compares them, so the description and its semantic
copy cannot drift. Six of twenty-one link pairs are disabled, and only pairs
rigidly attached to each other - a disable that has not been proved is not a
faster planner, it is an arm allowed to pass through itself. The acceleration
limit is derived from the effective inertia and costs 1.7% of the available
torque, in line with the 2% the drawing uses.

### What it found immediately

`move_group` loads the model, reports `Using position only ik`, solves inverse
kinematics onto the panel, and plans a joint-space motion in 10 ms.

It did not at first. Planning failed, and not because of the planner: **the arm
was in self-collision at the home pose** - the very pose every measured number
here is taken from.

The collision DAEs were transformed twice. `tools/align_collision_meshes.py`
baked the STL-to-DAE rotation into the vertices but left the COLLADA
`<node><matrix>` in place, and a loader applies it again. For `wrist_link` the
leftover 90 degrees about Y sent its 170 mm length down through the shoulder -
exactly the pair MoveIt reported. RViz and Gazebo draw the STL, so the arm had
always looked correct; nothing had ever read the collision geometry until now.

The two pictures below are the same RViz view of the **collision geometry
alone**, at the home pose, before and after. Nothing else changed between them.

| Before: the shape every collision query was reading | After: the shape the arm actually has |
|---|---|
| ![Collision geometry, transformed twice](docs/figures/15_collision_before.png) | ![Collision geometry, corrected](docs/figures/16_collision_after.png) |
| The wrist stands straight up as a bare cylinder, the links collapse into the base, and the tool mount with the pen floats detached in mid air. This is what the planner had been asked to reason about | The same query now returns the arm that is drawn, extended forward with the needle at the end |

The tool now does both halves of the job, five files were corrected, and no
published number moved - collision meshes feed neither the kinematics nor the
dynamics. `tests/test_meshes.py` compares the geometry a loader actually sees,
node transform included, and was checked by putting the defect back. See
[docs/moveit.md](docs/moveit.md#the-cause-the-collision-meshes-are-transformed-twice).

![MoveIt planning against the corrected arm](docs/figures/17_moveit_planning.png)

*`move_group` with the corrected geometry. Every link is the goal-state colour;
MoveIt paints a colliding link red, and there are none. With the old meshes this
same view came up red from the base to the pen, which is the defect above seen
from the planner's side.*

---

## Description Arguments

`tattotronix.urdf.xacro` takes five arguments:

| Argument | Values | Default | Effect |
|----------|--------|---------|--------|
| `hardware` | `none`, `mock`, `gz` | `none` | `none` omits ros2_control entirely, which is what RViz wants. `mock` loads `mock_components/GenericSystem` for testing controllers with no simulator. `gz` loads the Gazebo Sim plugin |
| `tool` | `tattoo`, `none` | `tattoo` | Whether the pen is mounted on `tool0` |
| `controllers_file` | path | empty | Controller manager YAML, read by the Gazebo plugin. Only consulted when `hardware:=gz` |
| `use_world_link` | `true`, `false` | `true` | Bolts `base_link` to a `world` link. Set false when the arm is embedded in a larger cell |
| `mass_model` | `printed`, `box` | `printed` | `printed` is the arm as built, shells plus servos, from `inertials_printed.xacro`; `box` is the earlier bounding-box approximation, kept for comparison. Anything else stops the build |

```bash
# the exported CAD with no tool and no control stack
xacro tattotronix.urdf.xacro hardware:=none tool:=none

# mock hardware, for bringing up controllers with nothing running
xacro tattotronix.urdf.xacro hardware:=mock
```

---

## Visualization

```bash
ros2 launch tattotronix_description display.launch.py
```

`robot_state_publisher` turns the xacro into TF and `joint_state_publisher_gui`
supplies the joint angles. No simulator, no controllers. This is the launch file
to reach for when the question is whether the kinematic tree is right.

<div align="center">
<img src="docs/images/rviz_display.png" width="90%"/>
</div>

Arguments: `gui` (`true` for the sliders, `false` for the headless publisher),
`tool`, `use_rviz`, `use_sim_time`.

---

## Simulation

```bash
ros2 launch tattotronix_gazebo simulation.launch.py
```

Gazebo Sim Fortress, the arm spawned on a bench under `ros2_control`, with RViz
alongside. One file owns the whole simulated robot: simulator, description,
spawn, clock bridge and controllers.

<div align="center">
<img src="docs/images/gazebo_simulation.png" width="90%"/>
</div>

The `studio` world is deliberately bare. A ground plane, a light, a 0.70 x 0.50 m
bench at the standard 750 mm height, and a 20 x 14 cm panel standing in for the
work surface, centred at x = 0.21 m. The bench is sized to the arm rather than
to a room: anything wider is scenery the robot can never touch.

The panel is centred under the tool flange, not under the tip. Now that the tool
axis runs along the bracket's +x, the zero pose puts the tip at x = 259 mm,
which is 49 mm forward of the panel centre and 9 mm off its centre line, still
well inside the 200 x 140 mm panel. Centring the panel on the tip would mean
moving it to x = 0.259 m.

The panel is rigid. Modelling compliant tissue is a separate piece of work and
pretending otherwise here would hide it.

Arguments: `world`, `world_name`, `tool`, `use_rviz`, `rviz_config`, `headless`, `spawn_z`.

---

## Control

```bash
ros2 control list_controllers
```

```
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active
arm_controller           joint_trajectory_controller/JointTrajectoryController  active
```

`tattotronix_control/config/tattotronix_controllers.yaml` is read by the Gazebo
plugin in simulation and will be read by `ros2_control_node` on hardware, so the
controller set cannot drift between the two. Anything that genuinely differs
belongs in the hardware interface, not in that file.

The controller manager runs at 1 kHz. It used to run at 200 Hz, on the
reasoning that a needle moving a few millimetres per second does not need more;
that reasoning was wrong. The loop rate sets the highest bandwidth a joint loop
can be tuned to, usable up to about a quarter of it, and the bandwidth is what
keeps the needle on the line — see
[the rate study](docs/mathematical-model/control.md#7-the-rate-is-the-ceiling).

`arm_controller` claims the position command interface only, because that is the
only command interface the description exposes; claiming another would leave the
controller unable to activate. Partial goals are refused, since on an arm
carrying a needle, leaving unnamed joints wherever they happen to be is never
what the caller meant.

---

## Migration Notes

The ROS 1 workspace this repository started from was ROS Industrial training
material: `myworkcell_core`, `myworkcell_support` and `myworkcell_moveit_config`
are a tutorial cell built around a UR5 that is not part of this project,
`fake_ar_publisher` is its stand-in perception node, and the vendored
`joint_state_publisher` packages are ROS 1 forks of packages Humble ships. All
of it was removed on the way to ROS 2, and none of it is lost: the history is
still in this branch, and `git show 19fa6ae` is the ROS 1 tree as it stood in
February 2022.

What carried over is the CAD and the joint geometry. Four things had to be
corrected on the way, because ROS 1 tolerated them and ROS 2 does not:

- **Zero effort limits.** Every joint declared `effort="0"`, which caps actuator
  torque at zero and makes the arm inert under any effort-aware controller. The
  axes now carry a 20 Nm, 1.5 rad/s envelope with damping and friction — a
  placeholder, far above the roughly 1 N·m and 0.2 N·m the arm's MG996R and
  SG90 servos are rated at.
- **An undeclared `world` link.** The base joint parented `base_link` to a link
  that was never declared. The anchor is now explicit and optional.
- **No inertia anywhere.** Each link first got a bounding-box approximation.
  Those masses later turned out to imply a density above steel's; the default
  is now the arm as built, integrated from the meshes — see
  [mass properties](docs/mathematical-model/mass.md).
- **Collision meshes in the wrong frame.** The CAD was exported twice, STL for
  the visuals and COLLADA for the collision shapes, and the two runs did not
  agree on axes. Every DAE declares `Z_UP` while its coordinates are Y-up, so
  five of six links collided with a shape rotated 90 degrees about X relative to
  what was drawn; the wrist was rotated about Y, and three carried a translation
  on top. A URDF never checks that a link's two geometries describe the same
  object, so nothing reported it.

That last one is worth expanding on, because it was silent and because the fix
is reproducible. Each DAE has the same triangle count and the same surface area
as its STL, to 0.01 cm2, so the pair differ by a rigid transform and nothing
else. `tools/align_collision_meshes.py` recovers that transform, bakes it into
the DAE vertices and clears the COLLADA node matrix that instantiates the
geometry. The first version did only the first half, so a loader applied the
transform twice and the arm sat in self-collision at its home pose, which
nothing noticed until MoveIt refused to plan — see
[docs/moveit.md](docs/moveit.md#the-cause-the-collision-meshes-are-transformed-twice):

```bash
python3 tools/align_collision_meshes.py          # report only
python3 tools/align_collision_meshes.py --apply  # rewrite the DAE files
```

It verifies a candidate by placing the DAE triangle centroids on the STL ones
through a spatial hash at 0.2 mm, and applies only a transform that matches
every sampled triangle, so a mesh that is not simply misoriented is reported and
left alone. It is idempotent, and re-runnable when the CAD is exported again.

The tool mount needed more than a reframe. Its visual was an export of the part
rotated 45.05 degrees about y, which is what made the pen look bolted on askew.
A second export in the same CAD drop was the identical mesh correctly oriented:
same 5094 triangles, same 61.99 cm2, bounding boxes matching to 0.01 mm once the
rotation was undone. That export is now the tool mount.

---

## What Is Real and What Is Placeholder

The distinction matters, because a detailed placeholder invites people to
measure off it.

**Measured, from the CAD:** the six link meshes and the volumes integrated from
them, the five joint axes and their origins, the +-1.57 rad travel, the tool
mount bracket and the 12.0 mm SG90 clamp gap.

**Real, not a stand-in:** the artwork. The drawing is the official ROS logo,
rasterised from the SVG published by
[ros-infrastructure/artwork](https://github.com/ros-infrastructure/artwork). It
is fetched rather than vendored, because the mark is CC BY-NC 4.0 and covered by
the [ROS trademark policy](https://www.ros.org/blog/media/) while this repository
is Apache-2.0. Run `docs/scripts/fetch_artwork.sh`.

**Placeholder, to be replaced:**

| Item | What it is now | Replace with |
|------|----------------|--------------|
| Link inertias | Each link's printed shell, its volume integrated from the mesh, plus the servos mounted in it - five large, two SG90 - at catalogue masses. The volumes are measured; the print density (0.35 of solid PLA) and the servo figures are declared | Weighing the parts once the arm is rebuilt |
| Joint effort and velocity limits | 20 Nm, 1.5 rad/s on every axis — far above the roughly 1 N·m and 0.2 N·m of the servos the arm carried | Limits from the servos actually fitted, per axis |
| The tattoo pen | One 45 mm x 3 mm cylinder on the tool axis | The pen, once it is built |
| `tool0` offset | x from the far +x face of the bracket mesh, y and z from the middle hole of the outer face and the bearing seat behind it | A CAD datum |
| Work surface | A rigid panel | A compliant tissue model, which is its own project |
| Plunge depth and marking feed | 1.5 mm below the surface at 6 mm/s | Whatever real tissue turns out to need |

**Where the model stands.** With gravity and velocity feedforward at
`wn = 160 rad/s` on a 1 kHz loop, a 4 mm slow approach into the work and the path
resampled at 0.15 mm, the tip holds 6.5 µm while marking and 46.8 µm at the worst
moment of the drawing, against a 0.3 mm tattoo line. Every modelled error term
is inside the line width. What stops this from being an accuracy claim is four
things: actuator torque, modelled only against a 20 N·m placeholder rather than
the servos the arm carried, and servo resolution, backlash and tissue, which are
not modelled at all. See
[control](docs/mathematical-model/control.md#6-recommended-configuration).

---

## Known Issues

- **The world can start paused.** `gz_args` carries `-r`, but the GUI's
  `WorldControl` plugin publishes its own run state at startup and can win that
  race. A paused world does not tick, a controller manager that does not tick
  never completes a switch, and the spawners fail on a five second timeout that
  is not theirs to configure. `simulation.launch.py` calls the Gazebo control
  service between the spawn and the spawners to settle it.
- **OSS CAD Suite shadows the ROS Python.** If `~/eda/oss-cad-suite/py3bin` is
  on `PATH` ahead of `/usr/bin`, `python3` resolves to its own 3.11 without
  PyYAML, and `colcon`, `xacro` and `ros2 launch` fail in confusing ways. Drop
  those entries from `PATH` in any shell used for ROS work.
- **A second publisher on `/joint_states` freezes the RViz model.** A leftover
  `joint_state_publisher` from some earlier launch keeps publishing zeros, and
  `robot_state_publisher` interleaves them with the real values from
  `joint_state_broadcaster`, so TF sits at the zero pose. The failure is
  confusing because Gazebo keeps moving — it has its own state and never reads
  the topic — and `/ink_trace` keeps growing, because `draw` draws the
  plan rather than the measured pose. Check with
  `ros2 topic info /joint_states`: the count should be one.
- **Nothing stops two simulations running at once.** Two of them fight over
  `/clock`, and the symptom is RViz logging "Detected jump back in time" while
  the controllers never finish activating. `docs/scripts/record_simulation.sh`
  refuses to start when it finds one already up; by hand, check with
  `pgrep -af "ign gazebo|rviz2|parameter_bridge"` before launching.
- **MoveIt plans, but not the drawing.** It is installed and configured, and it
  checks collisions and plans on request, but the drawing application still
  sends one precomputed trajectory, travel moves included. Handing MoveIt the
  travel moves is the next step, not a current feature.

---

## Usage

```bash
# Model only, joints on sliders
ros2 launch tattotronix_description display.launch.py

# Model with no tool
ros2 launch tattotronix_description display.launch.py tool:=none

# Simulation with RViz
ros2 launch tattotronix_gazebo simulation.launch.py

# Simulation with no GUI, for tests
ros2 launch tattotronix_gazebo simulation.launch.py headless:=true use_rviz:=false

# Check the description parses and the tree resolves
xacro src/tattotronix_description/urdf/tattotronix.urdf.xacro hardware:=none | check_urdf /dev/stdin
```

---

## Contributing

**[CONTRIBUTING.md](CONTRIBUTING.md)** has the whole of it: the develop → verify
→ test → certify → document → push cycle, the commit-message convention, what is
deliberately not versioned and why, and the two test suites.

The short version, before you push anything:

```bash
docker compose build ci                 # the ci service mounts nothing: build first
docker compose run --rm ci              # what continuous integration runs
python3 docs/scripts/check_docs.py      # published numbers still match the data
```

The `ci` service mounts nothing, which is the point — `dev` mounts the workspace
over the image, so a file missing from the image is still there and a broken
build passes. That has happened here.

---

## Acknowledgements

This project started in 2020 as an invitation from my robotics teacher **Olmer
García**, on an initiative by **Alis Paraquiva** and **Valeria Jorge**. It was
left unfinished when the pandemic took me out of the city, and picked up again
years later — which is what this repository is.

Thank you to the three of them. Without that first invitation it would never
have started.

The full account is in **[docs/project-history.md](docs/project-history.md)**,
including what the 2020 work was, what it became, and how the current version
was built.

---

## Contact

**Author**: Mario David Alvarez Vallejo
**Repository**: [github.com/MrDavidAlv/TATTOTRONIX](https://github.com/MrDavidAlv/TATTOTRONIX)
**License**: Apache 2.0 -- see [LICENSE](LICENSE)
