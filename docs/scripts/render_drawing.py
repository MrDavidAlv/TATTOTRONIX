"""Render the arm drawing the logo, from the solved trajectory.

This is not a screen capture. It is the same trajectory `draw_logo` sends to the
controller, drawn directly, and it says so on the frame. The distinction matters:
a screen capture of RViz would show the *commanded* path too, since the ink
marker is drawn from the plan, so rendering it here loses nothing and gains a
video anyone can regenerate without a compositor, a window manager or a
particular desktop.

What it does not show is tracking error. The tip follows the plan exactly here;
how well the real loop follows it is measured in
docs/mathematical-model/control.md, on the full non-linear plant.

    python3 docs/scripts/render_drawing.py              # mp4 + gif
    python3 docs/scripts/render_drawing.py --seconds 20

Writes docs/figures/drawing.mp4 and docs/figures/drawing.gif.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import rospath as rp
from analysis import LOGO_WIDTH_MM, PANEL_X, PANEL_Z, path_to_world, time_parameterise
from figures import C, GRID, INK, INK2, MUTED, SURFACE
from kinematics import Chain


def fit(box, lo, hi, pad=0.06):
    """Limits containing [lo, hi] whose aspect matches an axes box (w, h).

    With `aspect="equal"` matplotlib keeps the data square and letterboxes the
    rest, so limits chosen without reference to the box leave the content
    floating in a band of empty figure. Matching the aspect here is what makes
    the frame full.
    """
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    span = (hi - lo) * (1 + pad)
    mid = (hi + lo) / 2
    want = box[1] / box[0]
    if span[1] / span[0] < want:
        span[1] = span[0] * want
    else:
        span[0] = span[1] / want
    return (mid[0] - span[0] / 2, mid[0] + span[0] / 2,
            mid[1] - span[1] / 2, mid[1] + span[1] / 2)


FIG = Path(__file__).resolve().parents[1] / "figures"

# Two views rather than one isometric.
#
# A single isometric has to choose between reading the arm and reading the
# drawing: pitch the view up and the logo is legible while the arm needs a tall
# portrait frame, pitch it down and the logo shears into a stripe. Elevation
# plus a face-on panel gives both, and the logo comes out undistorted, which is
# the whole point of the video.


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=24.0,
                    help="length of the rendered video; the drawing is time "
                         "compressed to fit it")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--width", type=int, default=1000)
    args = ap.parse_args()

    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg is needed to write the video")

    chain = Chain.from_description()
    mask, grid = rp.ros_logo_mask(LOGO_WIDTH_MM)
    P_mm, kind = rp.toolpath(mask, grid)
    Pw = path_to_world(P_mm)
    t = time_parameterise(Pw, kind)
    print(f"path: {len(Pw)} points, {t[-1]:.0f} s", flush=True)

    q = np.zeros(chain.n)
    Q = np.zeros((len(Pw), chain.n))
    for i, p in enumerate(Pw):
        q, _, _ = chain.ik(p, [0, 0, -1], q)
        Q[i] = q
    n_frames = int(args.seconds * args.fps)
    # Frame k shows the drawing as it stands at this fraction of path time.
    stops = np.searchsorted(t, np.linspace(0.0, t[-1], n_frames))

    ink_uv = P_mm[:, :2]                       # panel coordinates, mm
    marked = np.concatenate([[False],
                             (kind[:-1] == rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK)])

    out = FIG / "frames"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    dpi = 100
    W = args.width
    fig = plt.figure(figsize=(W / dpi, W * 9 / 16 / dpi), dpi=dpi)
    ARM_BOX = (0.335, 0.76)
    PANEL_BOX = (0.545, 0.76)
    ax_arm = fig.add_axes([0.035, 0.055, *ARM_BOX])
    ax_panel = fig.add_axes([0.425, 0.055, *PANEL_BOX])
    # A figure box is a fraction of the figure, and the figure is 16:9, so the
    # box's own aspect has to carry that through.
    aspect = 9 / 16
    arm_box = (ARM_BOX[0], ARM_BOX[1] * aspect)
    panel_box = (PANEL_BOX[0], PANEL_BOX[1] * aspect)

    # Fixed limits, computed once: an axis that rescales as the drawing grows
    # makes the arm appear to move when it has not.
    reach = np.vstack([np.array([T[:3, 3] for _, T in chain.frames(Q[i])])
                       for i in range(0, len(Q), 40)]) * 1000.0
    arm_lim = fit(arm_box,
                  [min(reach[:, 0].min(), 0), -12],
                  [max(reach[:, 0].max(), (PANEL_X + 0.10) * 1000),
                   reach[:, 2].max()])
    panel_lim = fit(panel_box,
                    [-LOGO_WIDTH_MM / 2 - 10, -LOGO_WIDTH_MM / 2 - 10],
                    [LOGO_WIDTH_MM / 2 + 10, LOGO_WIDTH_MM / 2 + 10])
    # The panel is 200 x 140 mm; never show more of it than exists.
    panel_lim = (max(panel_lim[0], -100), min(panel_lim[1], 100),
                 max(panel_lim[2], -70), min(panel_lim[3], 70))

    from matplotlib.collections import LineCollection

    for k, stop in enumerate(stops):
        i = max(min(int(stop), len(Q) - 1), 0)
        seg = [ink_uv[j - 1:j + 1] for j in range(1, stop) if marked[j]]

        # --- arm, elevation in x-z
        ax_arm.clear()
        ax_arm.set_facecolor(SURFACE)
        pts = np.array([T[:3, 3] for _, T in chain.frames(Q[i])]) * 1000.0
        ax_arm.add_patch(plt.Rectangle(((PANEL_X - 0.10) * 1000, PANEL_Z * 1000 - 5),
                                       200, 5, facecolor="#efe3dd",
                                       edgecolor="#d8c4ba", zorder=1))
        ax_arm.axhline(0, color=GRID, lw=1.2, zorder=0)
        ax_arm.plot(pts[:, 0], pts[:, 2], "-", color=MUTED, lw=3.0,
                    solid_capstyle="round", zorder=3)
        ax_arm.scatter(pts[:, 0], pts[:, 2], s=30, color=C[0], zorder=4,
                       edgecolor=SURFACE, linewidth=1.3)
        ax_arm.scatter(pts[-1, 0], pts[-1, 2], s=24, color=C[1], zorder=5,
                       edgecolor=SURFACE, linewidth=1.1)
        ax_arm.set_xlim(arm_lim[0], arm_lim[1]); ax_arm.set_ylim(arm_lim[2], arm_lim[3])
        ax_arm.set_aspect("equal"); ax_arm.axis("off")
        ax_arm.set_title("the arm,  elevation", fontsize=9.5, loc="left",
                         color=INK2, pad=6)

        # --- panel, face on
        ax_panel.clear()
        ax_panel.set_facecolor("#efe3dd")
        if seg:
            ax_panel.add_collection(LineCollection(seg, colors=C[6], linewidths=1.5))
        ax_panel.scatter([P_mm[i, 0]], [P_mm[i, 1]], s=30, color=C[1], zorder=5,
                         edgecolor=SURFACE, linewidth=1.2)
        ax_panel.set_xlim(panel_lim[0], panel_lim[1])
        ax_panel.set_ylim(panel_lim[2], panel_lim[3])
        ax_panel.set_aspect("equal")
        ax_panel.set_xticks([]); ax_panel.set_yticks([])
        for sp in ax_panel.spines.values():
            sp.set_color("#d8c4ba")
        ax_panel.set_title("the panel,  face on", fontsize=9.5, loc="left",
                           color=INK2, pad=6)

        fig.text(0.035, 0.955, "TATTOTRONIX", fontsize=13, fontweight="bold",
                 color=INK, va="top")
        fig.text(0.035, 0.905,
                 f"t = {t[min(i, len(t) - 1)]:5.0f} s  of  {t[-1]:.0f} s     "
                 f"{'marking' if marked[min(i, len(marked) - 1)] else 'travel '}",
                 fontsize=9, color=INK2, va="top", family="DejaVu Sans Mono")
        fig.text(0.98, 0.955,
                 "the official ROS logo, 150 mm wide\n"
                 "rendered from the solved trajectory, not a screen capture",
                 fontsize=7.8, color=MUTED, ha="right", va="top", linespacing=1.6)

        fig.savefig(out / f"{k:05d}.png", facecolor=SURFACE)
        for txt in fig.texts[:]:
            txt.remove()
        if k % 50 == 0:
            print(f"  frame {k}/{n_frames}", flush=True)
    plt.close(fig)

    mp4, gif = FIG / "drawing.mp4", FIG / "drawing.gif"
    # yuv420p needs even dimensions and a figure size in inches rarely lands on
    # one, so the frames are trimmed to even rather than left to fail.
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
                    "-i", str(out / "%05d.png"),
                    "-vf", "crop=trunc(iw/2)*2:trunc(ih/2)*2",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                    str(mp4)], check=True)
    pal = out / "palette.png"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4),
                    "-vf", "fps=12,scale=760:-1:flags=lanczos,palettegen",
                    str(pal)], check=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4), "-i", str(pal),
                    "-lavfi", "fps=12,scale=760:-1:flags=lanczos[x];[x][1:v]paletteuse",
                    str(gif)], check=True)
    shutil.rmtree(out)
    print(f"wrote {mp4} ({mp4.stat().st_size/1e6:.1f} MB)")
    print(f"wrote {gif} ({gif.stat().st_size/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
