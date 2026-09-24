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

"""What the servos allow: the floor their resolution puts under the tip error.

Every control study here asks what a joint loop can achieve, and treats each
joint as something that takes a torque. The arm's servos do not. A hobby servo
takes a position, as the width of a pulse repeated every 20 ms, and closes its
own loop around its own potentiometer. Nothing above it reaches inside that
loop, and two things about it cannot be tuned away:

- its dead band: a change of command narrower than this does not move it at
  all, so it comes to rest anywhere within half the band of its command;
- the step of whatever generates the pulse, which rounds every command.

Through the arm's Jacobian both become an error at the needle that no
controller can remove. This computes it along the drawing for the servos the
arm was built with, driven two ways, and for bus servos that read their own
position; then turns the question around and asks what joint resolution the
drawing needs.

    python3 docs/scripts/actuator_study.py      # seconds

Writes data/actuator_study.json. It reads the exported trajectory rather than
the artwork, so it runs on a fresh clone.
"""

import itertools
import json
from pathlib import Path

import numpy as np

from kinematics import load
from mass_properties import SERVO_LAYOUT, SERVOS
from rospath import KIND_MARK, PLUNGE_DEPTH

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "data"
TRAJECTORY = ROOT / "src" / "tattotronix_control" / "config" / "trajectories" / "ros_logo.npz"

LINE_MM = 0.3       # the width of the line a tattoo needle lays; control.md's tolerance
STRIDE = 5          # every fifth marking pose; the path is sampled every 0.15 mm
SERIES_EVERY = 4    # of those, every fourth goes into the file for the figure

#: How far a hobby servo turns per microsecond of pulse. The usual convention
#: for a 180-degree servo is 500 to 2500 us from end to end; the catalogues for
#: these two parts do not state it. DECLARED: measure it on the real servos.
US_PER_DEG = 2000.0 / 180.0

#: What generates the pulses, and the step every command is rounded to. A
#: PCA9685 divides its 50 Hz frame into 4096 counts; Arduino's Servo library
#: takes whole microseconds.
PCA9685_STEP_US = 1e6 / 50.0 / 4096
ARDUINO_STEP_US = 1.0

#: A bus servo with a 12-bit encoder reads its position to one count in 4096 per
#: turn, and is assumed here to hold it to within one count. How closely one
#: really does depends on its dead-zone setting, which is configurable and not
#: modelled. DECLARED.
BUS_STEP_DEG = 360.0 / 4096


def servo_of_joint():
    """{joint: servo kind}, from the layout the mass model is built on.

    Two large servos share joint_2. They rest where the pair balances, and are
    modelled as one servo with the same dead band.
    """
    out = {}
    for mounts in SERVO_LAYOUT.values():
        for kind, joint in mounts:
            if joint.startswith("joint_"):
                out[joint] = kind
    return out


def hobby(step_us, kinds, names):
    """How far from its command each joint can come to rest, degrees either way.

    The command is rounded to the nearest step, so it is off by up to half of
    one, and the servo stops anywhere inside its dead band, which is centred on
    the command: up to half its width either way. The two add.
    """
    return np.array([(step_us + SERVOS[kinds[n]]["deadband_us"]) / 2 / US_PER_DEG
                     for n in names])


def lateral_worst(J_xy, e):
    """The largest in-panel tip error over the box of joint errors, at each pose.

    J_xy is (poses, 2, joints) in mm per degree, e the half-width of each joint's
    box in degrees. |J e| is convex, so its largest value over the box is at one
    of the box's corners, and there are only 2**joints of them.
    """
    corners = np.array(list(itertools.product((-1.0, 1.0), repeat=len(e))))
    tips = np.einsum("pij,cj->pci", J_xy, corners * e)
    return np.linalg.norm(tips, axis=2).max(axis=1)


def study():
    """Everything actuator_study.json holds, computed from the description."""
    chain = load()
    names = [s["name"] for s in chain.actuated]
    kinds = servo_of_joint()
    as_built = hobby(PCA9685_STEP_US, kinds, names)
    bus = np.full(len(names), BUS_STEP_DEG)
    # The joints that move the needle most, per section 4 of actuators.md: bus
    # servos there, the wrist left as built.
    upgraded = np.where(np.isin(names, ["joint_1", "joint_2", "joint_3"]), bus, as_built)
    options = {
        "as built, PCA9685 at 50 Hz": as_built,
        "as built, Arduino Servo library": hobby(ARDUINO_STEP_US, kinds, names),
        "bus servos on joints 1 to 3": upgraded,
        "bus servos, 12-bit encoders": bus,
    }

    d = np.load(TRAJECTORY)
    marking = np.flatnonzero(d["kind"] == KIND_MARK)[::STRIDE]
    Q = d["q"].astype(float)[marking]
    t = d["t"].astype(float)[marking]

    # Tip motion per degree at each joint: lateral is in the panel, which is
    # what moves the line; depth is along the needle.
    J = np.array([chain.jacobian(q)[:3] for q in Q]) * np.deg2rad(1.0) * 1000.0   # mm/deg
    lever_xy = np.linalg.norm(J[:, :2], axis=1)
    lever_z = np.abs(J[:, 2])

    # One degree of play at every joint, the worst way round: what any uniform
    # joint error - resolution, dead band or backlash - costs at worst, per degree.
    play = lateral_worst(J[:, :2], np.ones(len(names)))
    out = {
        "declared": {
            "us_per_deg": US_PER_DEG,
            "pca9685_step_us": PCA9685_STEP_US,
            "arduino_step_us": ARDUINO_STEP_US,
            "bus_step_deg": BUS_STEP_DEG,
            "servo_by_joint": {n: SERVOS[kinds[n]]["part"] for n in names},
            "deadband_us": {s["part"]: s["deadband_us"] for s in SERVOS.values()},
        },
        "line_mm": LINE_MM,
        "plunge_depth_mm": PLUNGE_DEPTH,
        "poses": int(len(Q)),
        "joints": names,
        "lever_lateral_mm_per_deg": lever_xy.max(axis=0).tolist(),
        "lever_depth_mm_per_deg": lever_z.max(axis=0).tolist(),
        "play_lateral_mm_per_deg": float(play.max()),
        "required_deg": LINE_MM / float(play.max()),
        "series_t_s": np.round(t[::SERIES_EVERY], 2).tolist(),
        "options": {},
    }
    for name, e in options.items():
        worst = lateral_worst(J[:, :2], e)
        rms = np.sqrt((lever_xy ** 2 * e ** 2 / 3).sum(axis=1))
        depth = (lever_z * e).sum(axis=1)
        out["options"][name] = {
            "rest_deg": e.tolist(),
            "lateral_worst_mm": float(worst.max()),
            "lateral_worst_median_mm": float(np.median(worst)),
            "lateral_rms_median_mm": float(np.median(rms)),
            "depth_worst_mm": float(depth.max()),
            "times_line": float(worst.max() / LINE_MM),
            "series_lateral_worst_mm": np.round(worst[::SERIES_EVERY], 4).tolist(),
        }
    return out


def main():
    out = study()
    print(f"{out['poses']} marking poses; one degree of play at every joint moves the "
          f"tip up to {out['play_lateral_mm_per_deg']:.2f} mm in the panel")
    print(f"to stay inside a {LINE_MM} mm line, every joint must hold to "
          f"{out['required_deg']:.4f} degrees")
    for name, r in out["options"].items():
        print(f"  {name:32}  rest within ±{max(r['rest_deg']):.3f} deg; lateral worst "
              f"{r['lateral_worst_mm']:.2f} mm ({r['times_line']:.1f} lines), typical "
              f"{r['lateral_rms_median_mm']:.2f} mm; depth worst {r['depth_worst_mm']:.2f} mm")
    (OUT / "actuator_study.json").write_text(json.dumps(out, indent=2))
    print("\nwrote", OUT / "actuator_study.json")


if __name__ == "__main__":
    main()
