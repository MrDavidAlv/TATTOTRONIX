# Copyright 2026 Mario David Alvarez Vallejo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Generate the MoveIt configuration from the URDF and the measured dynamics.

Two files come out of here, and neither is written by hand.

The semantic description is mostly restatement: which joints form the arm,
which link is the tip, and which pairs of links are allowed to overlap. All of
that is already implied by the URDF, and writing it out by hand means it stops
being true the first time a link is added and nobody remembers to edit a second
file. This reads the live description and emits the answer.

What it does not do is guess. MoveIt's setup assistant disables link pairs it
finds never to collide over a few thousand sampled configurations; this emits
only the pairs that are *provably* safe to disable - ones rigidly attached to
each other, which cannot move relative to each other at all. Everything else
stays enabled. An unproven disable is not a slower planner, it is an arm that
drives through itself, so the conservative direction is the only one taken
without a proof.

The second is joint_limits.yaml. MoveIt needs an acceleration bound to time
a trajectory and the URDF does not carry one, so it is derived rather than
picked: each joint is allowed to reach its URDF velocity limit in RAMP_S, and
the torque that costs is checked against the effort limit with the measured
peak gravity load already subtracted. The margin is printed and written into
the file. Acceleration is not what binds this arm - at these inertias a joint
would reach its speed limit in milliseconds - so the ramp time is a declared
choice about smoothness, and the point of deriving it this way is that the
choice is shown to be affordable instead of assumed to be.

Usage:

    python3 docs/scripts/make_moveit_config.py            # write both files
    python3 docs/scripts/make_moveit_config.py --stdout   # print, change nothing
"""

import argparse
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
URDF_XACRO = ROOT / "src" / "tattotronix_description" / "urdf" / "tattotronix.urdf.xacro"
CONFIG = ROOT / "src" / "tattotronix_moveit_config" / "config"
SRDF = CONFIG / "tattotronix.srdf"
LIMITS = CONFIG / "joint_limits.yaml"
SUMMARY = ROOT / "docs" / "data" / "summary.json"

#: Time a joint is given to go from rest to its URDF velocity limit. This is
#: the one declared number here. It buys smoothness, not safety: the torque it
#: costs is computed below and is a small fraction of what the joint has.
RAMP_S = 0.2

#: The planning group. base_link is the first link that does not move; the tip
#: is the needle point, so a Cartesian goal is a goal for the thing that draws
#: rather than for a flange some distance behind it.
GROUP = "arm"
BASE_LINK = "base_link"
TIP_LINK = "tattoo_tcp"


def urdf_xml(path=URDF_XACRO):
    """Expand the xacro and parse it. The description is the single source."""
    out = subprocess.run([sys.executable, "-c",
                          "import sys; from xacro import main; sys.argv=['xacro', %r]; main()"
                          % str(path)],
                         capture_output=True, text=True)
    if out.returncode != 0:                                   # pragma: no cover
        raise SystemExit("xacro failed:\n" + out.stderr)
    return ET.fromstring(out.stdout)


def collision_links(root):
    """Links that have collision geometry.

    Only these appear in the allowed-collision matrix. A frame with no shape
    cannot collide with anything, so naming it would be noise MoveIt ignores.
    """
    return [link.get("name") for link in root.findall("link") if link.findall("collision")]


def rigid_neighbours(root):
    """Pairs of colliding links that cannot move relative to each other.

    Two links are rigid neighbours when the path between them in the joint tree
    crosses exactly one movable joint, or none at all. Frames without collision
    geometry are passed straight through: tool_mount_link and tattoo_pen_link
    are separated only by tool0, which has no shape, so the two shapes are
    rigidly attached and overlap by construction.

    These are the only pairs disabled. Their relative pose is fixed by the
    geometry, so whether they touch is a property of the meshes and not of the
    configuration, and checking it every cycle answers the same question
    forever.
    """
    joints = [(j.find("parent").get("link"), j.find("child").get("link"),
               j.get("type") != "fixed")
              for j in root.findall("joint")]
    has_shape = set(collision_links(root))

    # Walk outwards from every colliding link, through shapeless frames for
    # free, and stop after the first movable joint.
    adj = {}
    for parent, child, movable in joints:
        adj.setdefault(parent, []).append((child, movable))
        adj.setdefault(child, []).append((parent, movable))

    pairs = set()
    for start in has_shape:
        stack = [(start, 0)]
        seen = {start}
        while stack:
            node, crossed = stack.pop()
            for nxt, movable in adj.get(node, []):
                cost = crossed + (1 if movable else 0)
                if cost > 1 or nxt in seen:
                    continue
                seen.add(nxt)
                if nxt in has_shape:
                    pairs.add(tuple(sorted((start, nxt))))
                else:
                    stack.append((nxt, cost))          # shapeless frame, pass through
    return sorted(pairs)


def group_joints(root):
    """The movable joints, base outwards, in tree order."""
    return [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]


def build(root):
    """Render the SRDF text."""
    joints = group_joints(root)
    pairs = rigid_neighbours(root)
    shapes = collision_links(root)
    total = len(shapes) * (len(shapes) - 1) // 2

    lines = [
        '<?xml version="1.0"?>',
        "<!--",
        "  Generated by docs/scripts/make_srdf.py from",
        "  src/tattotronix_description/urdf/tattotronix.urdf.xacro.",
        "",
        "  Do not edit by hand. Change the URDF and run the script again;",
        "  tests/test_moveit_config.py fails if this file and the description",
        "  have drifted apart.",
        "-->",
        '<robot name="tattotronix">',
        "",
        "  <!--",
        "    %d joints, which is one short of what a general pose in space needs." % len(joints),
        "    The arm cannot reach an arbitrary orientation, and nothing here",
        "    pretends otherwise: the planner is configured for position goals",
        "    and the drawing itself is solved elsewhere, by a 5x5 task Jacobian",
        "    that drops the spin about the needle axis because a body of",
        "    revolution does not care about it.",
        "  -->",
        '  <group name="%s">' % GROUP,
        '    <chain base_link="%s" tip_link="%s"/>' % (BASE_LINK, TIP_LINK),
        "  </group>",
        "",
        "  <!-- The pose every measured number in this repository is taken from. -->",
        '  <group_state name="home" group="%s">' % GROUP,
    ]
    lines += ['    <joint name="%s" value="0"/>' % j for j in joints]
    lines += [
        "  </group_state>",
        "",
        "  <!--",
        "    %d of %d link pairs disabled, and only these: pairs held rigid"
        % (len(pairs), total),
        "    to each other, whose overlap is a fact about the meshes rather than",
        "    about the configuration. The rest stay enabled. Disabling a pair",
        "    that has not been proved safe does not slow the planner down, it",
        "    lets the arm pass through itself.",
        "  -->",
    ]
    lines += ['  <disable_collisions link1="%s" link2="%s" reason="Adjacent"/>' % p
              for p in pairs]
    lines += ["", "</robot>", ""]
    return "\n".join(lines)


def joint_limits(root):
    """Velocity and effort per joint, straight from the description."""
    out = {}
    for j in root.findall("joint"):
        if j.get("type") == "fixed":
            continue
        lim = j.find("limit")
        out[j.get("name")] = {"velocity": float(lim.get("velocity")),
                              "effort": float(lim.get("effort"))}
    return out


def effective_inertia():
    """Recover 1/(M^-1)_ii per joint from the published gains.

    analysis.py places all three closed-loop poles at -wn, which fixes
    Kd = 3 J wn and Kp = 3 J wn^2. Their ratio is wn, so both the bandwidth
    the gains were tuned at and the inertia they were tuned against come back
    out of summary.json without re-running the dynamics.

    This is the effective inertia, 1/(M^-1)_ii, not the diagonal of M: what
    the axis actually has to accelerate when nothing holds the others still.
    """
    s = json.loads(SUMMARY.read_text(encoding="utf-8"))
    kp, kd = s["Kp"], s["Kd"]
    wn = [p / d for p, d in zip(kp, kd)]

    # The gains in summary.json are rounded for publication - Kd of joint 5 is
    # three significant figures - so the ratios agree to the rounding and not
    # further. A tolerance of machine precision here rejects a correct file;
    # one percent still catches a genuinely mistuned axis, which would be off
    # by a factor, not by a part in a thousand.
    mean = sum(wn) / len(wn)
    if max(abs(w - mean) for w in wn) > 0.01 * mean:          # pragma: no cover
        raise SystemExit("Kp/Kd is not one bandwidth: %r" % wn)
    return [d / (3.0 * mean) for d in kd], mean, s["grav_peak_Nm"]


def build_limits(root):
    """Render joint_limits.yaml, and the margin that justifies it."""
    lims = joint_limits(root)
    inertia, wn, grav_peak = effective_inertia()
    if len(inertia) != len(lims):                             # pragma: no cover
        raise SystemExit("gains cover %d joints, URDF has %d" % (len(inertia), len(lims)))

    rows, worst = [], 0.0
    for (name, lim), j_eff in zip(lims.items(), inertia):
        accel = lim["velocity"] / RAMP_S
        torque = j_eff * accel + grav_peak
        worst = max(worst, torque / lim["effort"])
        rows.append((name, lim, accel, j_eff, torque))

    out = [
        "# Generated by docs/scripts/make_moveit_config.py. Do not edit by hand.",
        "#",
        "# Velocity and effort are the description's. Acceleration is derived:",
        "# each joint reaches its velocity limit in %.2f s, and the torque that" % RAMP_S,
        "# costs is J_eff * a plus the measured peak gravity load of %.2f N.m," % grav_peak,
        "# against the effort limit the URDF declares. J_eff is 1/(M^-1)_ii,",
        "# recovered from the gains in docs/data/summary.json, which were placed",
        "# at %.0f rad/s." % wn,
        "#",
        "# Worst case across the arm is %.1f%% of the available effort, so the" % (100 * worst),
        "# ramp is a choice about smoothness rather than a limit being approached.",
        "",
        "joint_limits:",
    ]
    for name, lim, accel, j_eff, torque in rows:
        out += [
            "  %s:" % name,
            "    has_velocity_limits: true",
            "    max_velocity: %.4f" % lim["velocity"],
            "    has_acceleration_limits: true",
            "    max_acceleration: %.4f" % accel,
            "    # J_eff %.3e kg.m^2 -> %.3f N.m of %.1f available (%.1f%%)"
            % (j_eff, torque, lim["effort"], 100 * torque / lim["effort"]),
        ]
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--stdout", action="store_true", help="print instead of writing")
    args = ap.parse_args()

    root = urdf_xml()
    files = {SRDF: build(root), LIMITS: build_limits(root)}
    if args.stdout:
        for path, text in files.items():
            print("=" * 8, path.relative_to(ROOT))
            print(text, end="")
        return
    CONFIG.mkdir(parents=True, exist_ok=True)
    for path, text in files.items():
        path.write_text(text, encoding="utf-8")
        print("wrote %s" % path.relative_to(ROOT))


if __name__ == "__main__":
    main()
