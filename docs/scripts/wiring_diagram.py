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

"""The real arm's electronics, as a wiring diagram.

A Raspberry Pi 3 Model B+ drives a PCA9685 over I2C; the PCA9685 drives seven
servos, powered from a supply of their own. Which channel drives which joint,
and which servo turns it, are read from the same files the driver and the mass
model use - servo_calibration.yaml and mass_properties.py - so the drawing
cannot disagree with the arm it describes.

    python3 docs/scripts/wiring_diagram.py

Writes docs/figures/31_wiring.png and 31_wiring.svg.
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
import yaml

from mass_properties import SERVO_LAYOUT, SERVOS
from style import C, INK, INK2, MUTED, SURFACE
from style import apply as apply_style

ROOT = Path(__file__).resolve().parents[2]
FIGURES = ROOT / "docs" / "figures"
CALIBRATION = ROOT / "src" / "tattotronix_description" / "config" / "servo_calibration.yaml"

# Wire colours, by the conventions a reader will already know: red for supply,
# black for ground, blue and yellow for I2C data and clock as Qwiic cables have
# them, and a hobby servo's own cable - orange signal, red positive, brown
# negative.
POWER = C[7]
GROUND = INK
LOGIC = C[6]
SDA = C[0]
SCL = C[3]
SIGNAL = C[1]
BROWN = "#6b4226"
USB = "#5a5955"
BOARD = "#f1f0ec"
BAND = "#3b3a37"

NAMES = {"joint_1": "base", "joint_2": "shoulder", "joint_3": "elbow",
         "joint_4": "wrist roll", "joint_5": "wrist pitch"}


def servo_parts():
    """{joint: part}, from the layout the mass model is built on."""
    out = {}
    for mounts in SERVO_LAYOUT.values():
        for kind, joint in mounts:
            out[joint] = SERVOS[kind]["part"]
    return out


def channels():
    """(channel, joint, role) for every servo, in channel order."""
    cal = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    rows = []
    for joint, s in cal["servos"].items():
        rows.append((s["channel"], joint, "A" if "channel_b" in s else ""))
        if "channel_b" in s:
            rows.append((s["channel_b"], joint, "B"))
    rows.append((cal["tool"]["channel"], "tool", ""))
    return sorted(rows)


def box(ax, x, y, w, h, title, lines=(), fill=BOARD):
    """A component: an outlined board with its name on a dark band."""
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.6",
                                fc=fill, ec=INK2, lw=1.2, zorder=2))
    ax.add_patch(FancyBboxPatch((x, y + h - 9), w, 9, boxstyle="round,pad=0,rounding_size=1.6",
                                fc=BAND, ec=BAND, lw=1.2, zorder=3))
    ax.add_patch(Rectangle((x, y + h - 9), w, 4, fc=BAND, ec="none", zorder=3))
    ax.text(x + 3, y + h - 4.6, title, color="white", fontsize=10.5, weight="bold",
            va="center", zorder=4)
    for i, line in enumerate(lines):
        ax.text(x + 3, y + h - 14.5 - 5.2 * i, line, color=INK2, fontsize=8.2,
                va="center", zorder=4)


def pin(ax, x, y, colour=INK2, size=2.2):
    ax.add_patch(Rectangle((x - size / 2, y - size / 2), size, size, fc=colour,
                           ec=INK, lw=0.6, zorder=5))


def wire(ax, points, colour, lw=2.0, ls="-", z=4):
    xs, ys = zip(*points)
    ax.plot(xs, ys, color=colour, lw=lw, ls=ls, solid_capstyle="round",
            solid_joinstyle="round", zorder=z)


def servo(ax, x, y, muted=False):
    """A hobby servo seen from above: body, mounting ears, output spline, horn."""
    ec = MUTED if muted else INK2
    fc = "#f7f6f3" if muted else "#e9e7e1"
    ax.add_patch(Rectangle((x - 3, y - 1.6), 26, 3.2, fc=fc, ec=ec, lw=0.9, zorder=3))
    ax.add_patch(FancyBboxPatch((x, y - 4.8), 20, 9.6, boxstyle="round,pad=0,rounding_size=1",
                                fc=fc, ec=ec, lw=1.1, zorder=4))
    ax.add_patch(FancyBboxPatch((x + 7.5, y - 1.3), 14, 2.6,
                                boxstyle="round,pad=0,rounding_size=1.3",
                                fc="white", ec=ec, lw=1.0, zorder=5))
    ax.add_patch(Circle((x + 14.5, y), 2.1, fc="white", ec=ec, lw=1.0, zorder=6))
    ax.add_patch(Circle((x + 14.5, y), 0.7, fc=ec, ec="none", zorder=7))


def frame(ax, W, H):
    """A drawing sheet: border, and zone letters and numbers along it."""
    ax.add_patch(Rectangle((2, 2), W - 4, H - 4, fc="none", ec=INK, lw=1.6, zorder=1))
    ax.add_patch(Rectangle((6, 6), W - 12, H - 12, fc="none", ec=INK, lw=0.8, zorder=1))
    cols, rows = 8, 5
    for i in range(cols):
        x = 6 + (W - 12) * (i + 0.5) / cols
        for yy in (4, H - 4):
            ax.text(x, yy, str(i + 1), fontsize=6.5, color=INK2, ha="center", va="center")
        if i:
            xx = 6 + (W - 12) * i / cols
            for y0, y1 in ((2, 6), (H - 6, H - 2)):
                ax.plot([xx, xx], [y0, y1], color=INK, lw=0.6)
    for j in range(rows):
        y = 6 + (H - 12) * (rows - j - 0.5) / rows
        for xx in (4, W - 4):
            ax.text(xx, y, "ABCDE"[j], fontsize=6.5, color=INK2, ha="center", va="center")
        if j:
            yy = 6 + (H - 12) * j / rows
            for x0, x1 in ((2, 6), (W - 6, W - 2)):
                ax.plot([x0, x1], [yy, yy], color=INK, lw=0.6)


def draw():
    apply_style()
    W, H = 400.0, 250.0
    fig = plt.figure(figsize=(16, 10), facecolor=SURFACE)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")
    frame(ax, W, H)
    parts = servo_parts()
    rows = channels()

    # --- the PC, on the network -------------------------------------------
    ax.add_patch(FancyBboxPatch((22, 205), 34, 22, boxstyle="round,pad=0,rounding_size=1.2",
                                fc="#e9e7e1", ec=INK2, lw=1.2, zorder=2))
    ax.add_patch(Rectangle((25, 208), 28, 16, fc="white", ec=INK2, lw=0.8, zorder=3))
    ax.add_patch(Rectangle((35, 200), 8, 5, fc="#e9e7e1", ec=INK2, lw=1.0, zorder=2))
    ax.add_patch(Rectangle((29, 198), 20, 2.4, fc="#e9e7e1", ec=INK2, lw=1.0, zorder=2))
    ax.text(62, 222, "PC", fontsize=10.5, weight="bold", color=INK, va="center")
    ax.text(62, 215.5, "RViz · MoveIt · Gazebo", fontsize=8.2, color=INK2, va="center")
    ax.text(62, 209.5, "same network, same ROS_DOMAIN_ID", fontsize=8.2, color=INK2,
            va="center")

    # --- the Raspberry Pi ---------------------------------------------------
    px, py, pw, ph = 16, 92, 104, 76
    box(ax, px, py, pw, ph, "Raspberry Pi 3 Model B+",
        ["Ubuntu Server 22.04, 64-bit", "ROS 2 Humble · ros2_control",
         "arm_controller · Pca9685ServoSystem"])
    wire(ax, [(39, 198), (39, 168)], MUTED, lw=1.6, ls=(0, (4, 3)), z=1)
    ax.text(41, 183, "Wi-Fi or Ethernet\nROS 2 topics and actions", fontsize=7.6, color=INK2,
            va="center", linespacing=1.4)
    # its own supply
    ax.add_patch(FancyBboxPatch((16, 58), 44, 18, boxstyle="round,pad=0,rounding_size=1.2",
                                fc="white", ec=INK2, lw=1.1, zorder=2))
    ax.text(19, 70.5, "Pi supply", fontsize=9, weight="bold", color=INK, va="center")
    ax.text(19, 63.5, "5 V DC, 2.5 A, micro-USB", fontsize=7.8, color=INK2, va="center")
    wire(ax, [(38, 76), (38, 92)], USB, lw=2.6)
    ax.add_patch(Rectangle((34, 91), 8, 3, fc="#cfcdc6", ec=INK2, lw=0.8, zorder=5))
    ax.text(44, 84, "PWR IN", fontsize=7, color=INK2, va="center")

    # the four header pins used, in the order they run to the board
    j8 = [("6", "GND", GROUND), ("5", "GPIO 3 · SCL", SCL), ("3", "GPIO 2 · SDA", SDA),
          ("1", "3V3", LOGIC)]
    hx = px + pw - 3
    ax.add_patch(Rectangle((hx - 3.5, 98), 7, 38, fc="#2c2b28", ec=INK, lw=0.8, zorder=4))
    ax.text(hx - 5, 139.5, "J8 header", fontsize=7.6, color=INK2, ha="right", va="center")
    pin_y = {}
    for i, (number, label, colour) in enumerate(j8):
        y = 131 - 9.5 * i
        pin_y[label] = y
        pin(ax, hx, y, colour)
        ax.text(hx - 5, y, f"{label}  {number}", fontsize=7.8, color=INK, ha="right",
                va="center", zorder=6)

    # --- the PCA9685 ----------------------------------------------------------
    bx, by, bw, bh = 168, 64, 84, 140
    box(ax, bx, by, bw, bh, "PCA9685 · 16-channel PWM",
        ["I²C address 0x40", "50 Hz frame, 4096 counts: 4.88 µs"])
    # the logic header on its left edge
    header = [("GND", GROUND), ("OE", None), ("SCL", SCL), ("SDA", SDA), ("VCC", LOGIC),
              ("V+", None)]
    lx = bx + 2.5
    head_y = {}
    for i, (label, colour) in enumerate(header):
        y = 160 - 9.5 * i
        head_y[label] = y
        pin(ax, lx, y, colour or "white")
        ax.text(lx + 4, y, label + ("   n.c." if colour is None else ""), fontsize=7.8,
                color=INK if colour else MUTED, va="center", zorder=6)
    # Pi to board: each wire leaves its pin, jogs once, and lands
    for (label, colour), target, jog in (
            (("GND", GROUND), "GND", 142), (("GPIO 3 · SCL", SCL), "SCL", 147),
            (("GPIO 2 · SDA", SDA), "SDA", 152), (("3V3", LOGIC), "VCC", 157)):
        y0, y1 = pin_y[label], head_y[target]
        wire(ax, [(hx + 1.2, y0), (jog, y0), (jog, y1), (lx - 1.2, y1)], colour,
             lw=2.6 if colour == GROUND else 2.0)

    # the servo power terminal on its top edge, and the supply above it
    top = by + bh
    tx = [bx + 30, bx + 46]
    for x in tx:
        ax.add_patch(Rectangle((x - 4, top - 0.5), 8, 6, fc="#3a78c2", ec=INK, lw=0.8,
                               zorder=5))
        ax.add_patch(Circle((x, top + 2.5), 1.6, fc="#cfcdc6", ec=INK, lw=0.6, zorder=6))
    ax.text(tx[0] - 6, top + 2.5, "V+", fontsize=7.8, color=INK, ha="right", va="center")
    ax.text(tx[1] + 6, top + 2.5, "GND   servo power in", fontsize=7.8, color=INK,
            ha="left", va="center")
    sx, sy = bx + 12, 226
    ax.add_patch(FancyBboxPatch((sx, sy), 64, 15, boxstyle="round,pad=0,rounding_size=1.2",
                                fc="white", ec=INK2, lw=1.1, zorder=2))
    ax.text(sx + 3, sy + 10.2, "Servo supply, 6 V DC", fontsize=9, weight="bold", color=INK,
            va="center")
    ax.text(sx + 3, sy + 4.2, "for all seven servos stalling at once", fontsize=7.6,
            color=INK2, va="center")
    # + through S1 to V+, - straight to GND
    s_hi, s_lo = 221.5, 214.5
    wire(ax, [(tx[0], sy), (tx[0], s_hi)], POWER, lw=3.2)
    wire(ax, [(tx[0], s_lo), (tx[0], top + 4)], POWER, lw=3.2)
    ax.add_patch(Circle((tx[0], s_hi), 1.1, fc="white", ec=INK, lw=0.9, zorder=6))
    ax.add_patch(Circle((tx[0], s_lo), 1.1, fc="white", ec=INK, lw=0.9, zorder=6))
    wire(ax, [(tx[0], s_lo + 1.1), (tx[0] - 5.5, s_hi + 0.5)], INK, lw=1.5, z=6)
    ax.text(tx[0] - 8, (s_hi + s_lo) / 2, "S1  servo power", fontsize=7.8, color=INK,
            ha="right", va="center")
    wire(ax, [(tx[1], sy), (tx[1], top + 4)], GROUND, lw=3.2)

    # --- the servos --------------------------------------------------------------
    rx = bx + bw
    for k, (channel, joint, role) in enumerate(rows):
        y = 186 - 17.5 * k
        tool = joint == "tool"
        # the channel's 3-pin header: signal, positive, negative
        ax.add_patch(Rectangle((rx - 8.5, y - 4.4), 8, 8.8, fc="#2c2b28", ec=INK, lw=0.7,
                               zorder=4))
        for dy, colour in ((2.6, SIGNAL), (0.0, POWER), (-2.6, BROWN)):
            pin(ax, rx - 4.5, y + dy, "white" if tool else colour, size=1.8)
        ax.text(rx - 10.5, y, f"CH{channel}", fontsize=8, weight="bold",
                color=MUTED if tool else INK, ha="right", va="center", zorder=6)
        # the servo's own cable
        ls = (0, (3, 2)) if tool else "-"
        for dy, colour in ((1.3, SIGNAL), (0.0, POWER), (-1.3, BROWN)):
            wire(ax, [(rx - 3.5, y + dy), (rx + 26, y + dy)], MUTED if tool else colour,
                 lw=1.3, ls=ls)
        servo(ax, rx + 29, y, muted=tool)
        name = "tool" if tool else f"{joint} · {NAMES[joint]}"
        part = SERVOS["small"]["part"] if tool else parts[joint]
        detail = {"A": f"{part}, first of two",
                  "B": f"{part}, second, turning the other way"}.get(role, part)
        if tool:
            detail = f"{part}, continuous rotation · not driven yet"
        ax.text(rx + 57, y + 2.4, name, fontsize=9, weight="bold",
                color=MUTED if tool else INK, va="center")
        ax.text(rx + 57, y - 3.2, detail, fontsize=7.8, color=MUTED if tool else INK2,
                va="center")
    ax.text(rx - 10.5, 186 - 17.5 * len(rows) + 6, "CH7 to CH15 free", fontsize=7.6,
            color=MUTED, ha="right", va="center")
    # the shoulder's two servos, bracketed
    ya, yb = 186 - 17.5 * 1, 186 - 17.5 * 2
    wire(ax, [(W - 14, ya + 5), (W - 12, ya + 5), (W - 12, yb - 5), (W - 14, yb - 5)], INK2,
         lw=1.0)
    ax.text(W - 9.5, (ya + yb) / 2, "one joint", fontsize=7.2, color=INK2, rotation=90,
            ha="center", va="center")

    # --- legend, notes, title block ---------------------------------------------
    ax.add_patch(Rectangle((10, 10), 104, 40, fc="white", ec=INK2, lw=0.9, zorder=2))
    ax.text(13, 45.5, "WIRES", fontsize=8, weight="bold", color=INK, va="center")
    legend = [("Servo power, 6 V", POWER, 3.2, "-"), ("Ground", GROUND, 3.2, "-"),
              ("Logic, 3.3 V", LOGIC, 2.0, "-"), ("I²C data, SDA", SDA, 2.0, "-"),
              ("I²C clock, SCL", SCL, 2.0, "-"), ("Network", MUTED, 1.6, (0, (4, 3))),
              ("USB power, 5 V", USB, 2.6, "-")]
    for i, (label, colour, lw, ls) in enumerate(legend):
        col, row = i // 4, i % 4
        x, y = 13 + col * 52, 39 - row * 5.8
        wire(ax, [(x, y), (x + 10, y)], colour, lw=lw, ls=ls, z=3)
        ax.text(x + 13, y, label, fontsize=7.6, color=INK, va="center")
    for dy, colour in ((1.3, SIGNAL), (0.0, POWER), (-1.3, BROWN)):
        wire(ax, [(13, 15 + dy), (23, 15 + dy)], colour, lw=1.3, z=3)
    ax.text(26, 15, "Servo cable: signal (orange), + (red), − (brown)", fontsize=7.6,
            color=INK, va="center")

    ax.add_patch(Rectangle((118, 10), 170, 40, fc="white", ec=INK2, lw=0.9, zorder=2))
    ax.text(121, 45.5, "NOTES", fontsize=8, weight="bold", color=INK, va="center")
    notes = [
        "1  The servos take power from their own supply, never from the Raspberry Pi.",
        "2  One ground for everything: Pi, PCA9685 and servo supply.",
        "3  S1 cuts the servos' power. It is the way to stop the arm.",
        "4  The shoulder's two servos are declared as turning opposite ways. Check it,",
        "    calibrating each alone before both are connected: docs/hardware.md, section 5.",
        "5  Channels, joints and servos are read from servo_calibration.yaml.",
    ]
    for i, line in enumerate(notes):
        ax.text(121, 39 - 5.4 * i, line, fontsize=7.6, color=INK, va="center")

    tb = (292, 10, 98, 40)
    ax.add_patch(Rectangle(tb[:2], tb[2], tb[3], fc="white", ec=INK, lw=1.2, zorder=2))
    for y in (38, 26, 18):
        ax.plot([tb[0], tb[0] + tb[2]], [y, y], color=INK, lw=0.7, zorder=3)
    ax.plot([tb[0] + 49, tb[0] + 49], [10, 18], color=INK, lw=0.7, zorder=3)
    ax.text(tb[0] + 3, 44, "TATTOTRONIX", fontsize=11, weight="bold", color=INK,
            va="center")
    ax.text(tb[0] + 3, 34.5, "Servo electronics · wiring diagram", fontsize=8.6, color=INK,
            va="center")
    ax.text(tb[0] + 3, 29.5, "Pi 3 B+ · PCA9685 · 5 × MG996R · 2 × SG90", fontsize=7.4,
            color=INK2, va="center")
    ax.text(tb[0] + 3, 22, "docs/scripts/wiring_diagram.py", fontsize=7.4, color=INK2,
            va="center")
    ax.text(tb[0] + 3, 14, "Sheet 1 of 1", fontsize=7.4, color=INK2, va="center")
    ax.text(tb[0] + 52, 14, "Not to scale", fontsize=7.4, color=INK2, va="center")
    return fig


def write(folder):
    """Draw it into folder, as 31_wiring.png and 31_wiring.svg.

    The SVG carries no date and names its elements from a fixed salt, so the
    same drawing always gives the same file, and a test can hold the published
    one to what this script draws now.
    """
    plt.rcParams["svg.hashsalt"] = "tattotronix-wiring"
    fig = draw()
    folder.mkdir(parents=True, exist_ok=True)
    fig.savefig(folder / "31_wiring.png", dpi=200, facecolor=SURFACE)
    fig.savefig(folder / "31_wiring.svg", facecolor=SURFACE, metadata={"Date": None})
    plt.close(fig)


def main():
    write(FIGURES)
    print("wrote", FIGURES / "31_wiring.png", "and", FIGURES / "31_wiring.svg")


if __name__ == "__main__":
    main()
