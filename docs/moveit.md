# Planning with MoveIt

MoveIt was added for **collision checking and free-space motion**, not for
inverse kinematics. The reasoning is in
[docs/architecture.md](architecture.md#where-moveit-fits-and-where-it-does-not);
the short version is that five joints cannot reach an arbitrary six-number pose,
so a solver asked for a full pose fails on nearly every goal, while the drawing
itself is already solved better offline by this project's 5×5 task Jacobian.

| Motion | Planned by |
|---|---|
| Drawing a stroke | This project's 5×5 task inverse kinematics, offline |
| Lifting, travelling between strokes, approach and retract | MoveIt |
| Checking any of it is collision free | MoveIt's planning scene |

---

## Running it

Start the simulation first, then the planner. This launch brings up
`move_group` alone: no simulator, no controllers, no `robot_state_publisher`,
because a second copy of any of those is how two descriptions end up
disagreeing about where the arm is.

```bash
ros2 launch tattotronix_gazebo simulation.launch.py
ros2 launch tattotronix_moveit_config move_group.launch.py
```

`hardware:=gz|mock` must match what the running system was started with.

---

## The configuration, and which parts are derived

Nothing in `config/` is hand-written except the planner choice. Two files are
generated from the description and the measured dynamics:

```bash
python3 docs/scripts/make_moveit_config.py
```

`tests/test_moveit_config.py` regenerates both and compares, so a link added to
the URDF or a controller renamed fails the build rather than the first plan.

**`tattotronix.srdf`** — the group is the chain `base_link` → `tattoo_tcp`, so a
Cartesian goal is a goal for the needle point rather than for a flange some
distance behind it. Six of twenty-one link pairs are disabled, and only pairs
that are *rigidly attached to each other*, whose overlap is a property of the
meshes and not of the configuration. MoveIt's setup assistant also disables
pairs it samples and never finds colliding; this does not, because a disable
that has not been proved is not a faster planner, it is an arm allowed to pass
through itself.

**`joint_limits.yaml`** — velocity and effort come from the URDF. Acceleration
is not in the URDF, so it is derived rather than picked: each joint is given
0.2 s to reach its velocity limit, and the torque that costs is checked against
the effort limit with the measured peak gravity load of 1.91 N·m already
subtracted. Worst case across the arm is **10.6% of the available effort**, in
line with the 12% the drawing itself uses. The effective inertia it is computed
against, 1/(M⁻¹)ᵢᵢ, is recovered from the published gains rather than re-derived.
The ramp time is the one declared number, and deriving it this way is what shows
the choice to be affordable instead of assuming it.

**`kinematics.yaml`** — `position_only_ik: true`. Three constraints against five
joints, with two left redundant, which is well posed. This solver is for travel
and never draws.

---

## What it verified, and what it found

Run against the live description, `move_group` loads the model, reports
`Using position only ik`, and advertises `/plan_kinematic_path`, `/compute_ik`
and `/get_planning_scene`. Position-only inverse kinematics onto the middle of
the panel solves.

Joint-space planning does **not** yet succeed, and the reason is not the
planner. OMPL reports `Skipping invalid start state`: the arm is in
self-collision at the pose it starts from. Asked directly through
`/check_state_validity`, MoveIt names the pairs:

| Pose | Result |
|---|---|
| Home, all joints zero | **In collision**: `shoulder_link ↔ wrist_link`, `upper_arm_link ↔ wrist_link` |
| `joint_2 = ±0.5` | In collision, same pairs |
| `j2 = 0.5, j3 = -0.8` | Valid |
| A pose taken from the drawing | Valid |

### The cause: the collision meshes are transformed twice

This is a real defect in the description, and it had been invisible because
nothing before now checked the collision geometry against anything.

Each link is exported twice, an STL for visual and a COLLADA DAE for collision,
and the two exports did not agree on axes. `tools/align_collision_meshes.py`
found the rigid transform between them and **baked it into the DAE vertices**,
which is why the raw vertex bounding boxes and triangle counts of the two now
match exactly, link for link.

What it did not do is clear the `<node><matrix>` that instantiates the geometry
in each DAE's visual scene. Those matrices are still the original ones, and
assimp applies them on top of vertices that have already been corrected. The
transform lands twice:

| Link | Node matrix still present |
|---|---|
| `wrist_link` | 90° about Y |
| `forearm_link`, `upper_arm_link`, `shoulder_link` | −90° about X, two with a translation |
| `tool_mount_link` | 45°, with a translation |
| `base_link` | Identity — the one that is already correct |

For the wrist the consequence is exact and checkable by hand: its mesh runs from
0 to 170 mm along +x, and the leftover rotation sends that 170 mm down −z
instead. Its frame sits at about z = 242 mm, so the rotated shape reaches down
to roughly z = 72 mm, straight through the shoulder, which occupies z ∈ [51,
118] mm. That is precisely the pair MoveIt reports.

RViz and Gazebo draw the STL, so the arm has always *looked* right. Only a
collision check reads the DAE, and until now nothing did.

**The fix** is to set the geometry node's matrix to identity in the five
affected DAEs, since the vertices are already in the link frame, leaving the
camera and light nodes alone. It is not applied yet: it changes the geometry
every published number depends on, so it is a cycle of its own, with the
pen-tip and panel clearances re-measured afterwards.

Until it is, the honest statement is that **collision checking is configured and
working — it is the collision geometry that is wrong**, and MoveIt is what
found it.

---

## Known noise

`No 3D sensor plugin(s) defined for octomap updates` is logged at ERROR on
startup. There is no depth sensor on this robot and the occupancy monitor starts
regardless. It is expected and harmless.

---

## See also

- [docs/architecture.md](architecture.md) — why planning is a package of its own
- [docs/mathematical-model/kinematics.md](mathematical-model/kinematics.md) — the 5×5 task Jacobian that draws
- [CONTRIBUTING.md](../CONTRIBUTING.md) — the cycle
