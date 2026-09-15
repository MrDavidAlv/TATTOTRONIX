"""Every figure in DEVELOPMENT.md, from the cached analysis.

Palette and mark specs follow the project's data-viz rules: categorical hues in
fixed order and never cycled, one hue light-to-dark for magnitude, thin marks,
recessive grid, a legend whenever more than one series is on screen and direct
labels where they fit. Figures are written on an explicit light surface so they
read the same in a dark editor as in a browser.
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Rectangle

import rospath as rp
from analysis import PANEL_H, PANEL_W, PANEL_X, PANEL_Z

DATA = Path(__file__).resolve().parents[1] / "data"
FIG = Path(__file__).resolve().parents[1] / "figures"
FIG.mkdir(exist_ok=True)

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#8a8984"
GRID = "#e4e3df"

# categorical, fixed order
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
# sequential, one hue light -> dark
BLUES = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
SEQ = LinearSegmentedColormap.from_list("seq", BLUES)

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE, "savefig.dpi": 200, "figure.dpi": 200,
    "font.family": "DejaVu Sans", "font.size": 9,
    "text.color": INK, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "xtick.color": INK2, "ytick.color": INK2,
    "axes.edgecolor": GRID, "axes.linewidth": 0.8,
    "grid.color": GRID, "grid.linewidth": 0.7,
    "lines.linewidth": 1.6, "lines.solid_capstyle": "round",
    "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
})

def _wrap(text, width):
    import textwrap
    return "\n".join(textwrap.wrap(text, width))


D = np.load(DATA / "analysis.npz")
JOINTS = [f"joint_{i+1}" for i in range(5)]


SUB_WRAP = 118          # characters per subtitle line
SUB_LINE_IN = 0.185     # height of one subtitle line, inches


def finish(fig, name, title=None, sub=None):
    if title:
        fig.tight_layout()
        h = fig.get_size_inches()[1]
        # The title sits above however many lines the subtitle wraps to. Fixing
        # the offset instead lands the title on top of a three line subtitle.
        text = _wrap(sub, SUB_WRAP) if sub else ""
        lines = text.count("\n") + 1 if sub else 0
        base = 0.20 / h
        fig.text(0.0, 1.0 + base + (lines * SUB_LINE_IN + 0.22) / h, title,
                 ha="left", va="bottom", fontsize=12.5, fontweight="bold",
                 transform=fig.transFigure)
        if sub:
            fig.text(0.0, 1.0 + base, text, ha="left", va="bottom",
                     fontsize=8.8, color=INK2, linespacing=1.5,
                     transform=fig.transFigure)
    fig.savefig(FIG / name, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print("  ", name, flush=True)


def grid_on(ax, axis="both"):
    ax.grid(True, axis=axis, alpha=0.9, zorder=0)
    ax.set_axisbelow(True)


# --- 1. the arm at the zero pose, and while marking --------------------------

def fig_chain():
    import subprocess
    from kinematics import Chain
    chain = Chain.from_description()
    poses = [(np.zeros(5), "zero pose"), (D["path_q"][len(D["path_q"]) // 2], "marking the panel")]
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.6))
    for ax, (q, label) in zip(axes, poses):
        pts = np.array([T[:3, 3] for _, T in chain.frames(q)])
        ax.plot(pts[:, 0] * 1000, pts[:, 2] * 1000, "-", color=MUTED, lw=2.2, zorder=2)
        ax.scatter(pts[:, 0] * 1000, pts[:, 2] * 1000, s=26, color=C[0], zorder=3,
                   edgecolor=SURFACE, linewidth=1.2)
        # tool axis
        T = chain.fk(q)
        p, z = T[:3, 3] * 1000, T[:3, 2]
        ax.annotate("", xy=(p[0] + z[0] * 28, p[2] + z[2] * 28), xytext=(p[0], p[2]),
                    arrowprops=dict(arrowstyle="-|>", color=C[1], lw=1.8))
        ax.add_patch(Rectangle(((PANEL_X - PANEL_W / 2) * 1000, PANEL_Z * 1000 - 5),
                               PANEL_W * 1000, 5, facecolor="#efe3dd",
                               edgecolor="#d8c4ba", zorder=1))
        ax.axhline(0, color=GRID, lw=1.2)
        ax.set_title(label, fontsize=10, loc="left", pad=8)
        ax.set_xlabel("x  [mm]"); ax.set_aspect("equal")
        grid_on(ax)
    axes[0].set_ylabel("z  [mm]")
    axes[0].text(266, 250, "pen axis", color=C[1], fontsize=8.4)
    axes[1].annotate("panel", xy=((PANEL_X + 0.05) * 1000, 3), xytext=(268, 52),
                     color="#a8887a", fontsize=8.4,
                     arrowprops=dict(arrowstyle="-", color="#d8c4ba", lw=1))
    finish(fig, "01_chain.png", "Kinematic chain, elevation in the x-z plane",
           "Link frames along the chain and the pen axis. The panel sits 5 mm above the bench, "
           "which is the arm's own z = 0.")


# --- 2. workspace ------------------------------------------------------------

def fig_workspace():
    P = D["ws_P"] * 1000
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.4))
    for ax, (a, b, la, lb) in zip(axes, [(0, 2, "x", "z"), (0, 1, "x", "y")]):
        ax.hexbin(P[:, a], P[:, b], gridsize=68, cmap=SEQ, mincnt=1, linewidths=0)
        ax.set_xlabel(f"{la}  [mm]"); ax.set_ylabel(f"{lb}  [mm]")
        ax.set_aspect("equal")
    axes[0].add_patch(Rectangle(((PANEL_X - PANEL_W / 2) * 1000, PANEL_Z * 1000 - 5),
                                PANEL_W * 1000, 5, facecolor="none",
                                edgecolor=C[1], lw=1.8, zorder=5))
    axes[1].add_patch(Rectangle(((PANEL_X - PANEL_W / 2) * 1000, -PANEL_H / 2 * 1000),
                                PANEL_W * 1000, PANEL_H * 1000, facecolor="none",
                                edgecolor=C[1], lw=1.8, zorder=5))
    axes[0].text((PANEL_X) * 1000, 40, "panel", color=C[1], fontsize=8.6, ha="center")
    finish(fig, "02_workspace.png", "Reachable set of the pen tip",
           f"{len(P):,} poses drawn uniformly from the joint box, projected onto two planes. "
           "Darker cells are reached by more of joint space.")


# --- 3. panel reachability ---------------------------------------------------

def fig_panel():
    xs, ys = D["panel_xs"] * 1000, D["panel_ys"] * 1000
    ok, mu, margin = D["panel_ok"], D["panel_mu"], D["panel_margin"]
    fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.1))
    ext = [xs[0], xs[-1], ys[0], ys[-1]]

    im = axes[0].imshow(np.where(ok, mu, np.nan), origin="lower", extent=ext,
                        cmap=SEQ, aspect="equal")
    cb = fig.colorbar(im, ax=axes[0], fraction=0.035, pad=0.02)
    cb.set_label("manipulability", fontsize=8.2)
    cb.outline.set_visible(False)
    axes[0].set_title("how well conditioned", fontsize=10, loc="left", pad=8)

    im2 = axes[1].imshow(np.where(ok, np.rad2deg(margin), np.nan), origin="lower",
                         extent=ext, cmap=SEQ, aspect="equal")
    cb2 = fig.colorbar(im2, ax=axes[1], fraction=0.035, pad=0.02)
    cb2.set_label("margin to a joint limit  [deg]", fontsize=8.2)
    cb2.outline.set_visible(False)
    axes[1].set_title("how much travel is left", fontsize=10, loc="left", pad=8)

    lo, hi = D["mask_xs"], D["mask_ys"]
    w, h = lo[-1] - lo[0], hi[0] - hi[-1]
    for ax in axes:
        ax.add_patch(Rectangle((PANEL_X * 1000 - w / 2, -h / 2), w, h,
                               facecolor="none", edgecolor=C[1], lw=1.6, ls=(0, (4, 2))))
        ax.set_xlabel("x  [mm]"); ax.set_ylabel("y  [mm]")
    axes[0].text(PANEL_X * 1000, -h / 2 - 16, "artwork footprint", color=C[1],
                 fontsize=8.2, ha="center", va="top")
    pct = ok.mean() * 100
    finish(fig, "03_panel.png", "Can the needle stand normal to the panel?",
           f"Inverse kinematics solved on a grid over the work surface with the pen held along "
           f"-z. {pct:.1f}% of the panel admits a solution; white cells admit none.")


# --- 4. the toolpath ---------------------------------------------------------

def fig_toolpath():
    P, kind = D["path_mm"], D["path_kind"]
    mask, xs, ys = D["mask"], D["mask_xs"], D["mask_ys"]
    # The mark is nearly four times wider than it is tall, so the two views
    # stack. Side by side, `aspect="equal"` pads each one with more empty space
    # than drawing.
    fig, axes = plt.subplots(2, 1, figsize=(10.2, 5.4), sharex=True)

    axes[0].imshow(mask, origin="upper", extent=[xs[0], xs[-1], ys[-1], ys[0]],
                   cmap="gray_r", alpha=0.16, aspect="equal")
    seg = np.concatenate([[False], (kind[:-1] == 1) & (kind[1:] == 1)])
    for i in range(1, len(P)):
        if seg[i]:
            axes[0].plot(P[i - 1:i + 1, 0], P[i - 1:i + 1, 1], color=C[0], lw=0.6)
    axes[0].set_title("needle in the work", fontsize=10, loc="left", pad=8)

    axes[1].imshow(mask, origin="upper", extent=[xs[0], xs[-1], ys[-1], ys[0]],
                   cmap="gray_r", alpha=0.16, aspect="equal")
    for i in range(1, len(P)):
        if not seg[i]:
            axes[1].plot(P[i - 1:i + 1, 0], P[i - 1:i + 1, 1], color=C[1], lw=0.6, alpha=0.85)
    axes[1].set_title("travel, needle clear", fontsize=10, loc="left", pad=8)

    for ax in axes:
        ax.set_aspect("equal"); ax.set_ylabel("v  [mm]")
        grid_on(ax)
    axes[1].set_xlabel("u  [mm]")
    marked, travel = rp.lengths(P, kind)
    entries = int(((kind[:-1] == 0) & (kind[1:] == 1)).sum())
    finish(fig, "04_toolpath.png", "Toolpath over the artwork",
           f"The official ROS logo, rasterised from the SVG. Boundary pass then boustrophedon "
           f"fill at a {rp.STROKE_PITCH:g} mm stroke pitch: {marked:.0f} mm marked against "
           f"{travel:.0f} mm travelled, over {entries} separate needle entries. Every scanline "
           "that meets a counter has to lift and re-enter, which is where the travel goes.")


# --- 5. joint trajectories ---------------------------------------------------

def fig_joint_traj():
    t, Q, kind = D["path_t"], D["path_q"], D["path_kind"]
    fig, axes = plt.subplots(5, 1, figsize=(10.0, 7.0), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, np.rad2deg(Q[:, i]), color=C[i], lw=1.3)
        ax.set_ylabel("deg", fontsize=8.2)
        ax.text(0.004, 0.76, JOINTS[i], transform=ax.transAxes, color=C[i],
                fontsize=9, fontweight="bold")
        ax.axhline(90, color=MUTED, lw=0.8, ls=(0, (3, 3)))
        ax.axhline(-90, color=MUTED, lw=0.8, ls=(0, (3, 3)))
        grid_on(ax, "y")
    axes[-1].set_xlabel("time  [s]")
    axes[0].text(t[-1], 92, "joint limit  +-90 deg", color=MUTED, fontsize=7.6,
                 ha="right", va="bottom")
    finish(fig, "05_joint_trajectories.png", "Joint angles over the whole sheet",
           f"Inverse kinematics along the {t[-1]:.0f} s path, one panel per axis so no two "
           "traces share a scale. Dashed lines are the travel limits.")


# --- 6. manipulability along the path ----------------------------------------

def fig_manip():
    t, mu, kind = D["path_t"], D["path_mu"], D["path_kind"]
    fig, ax = plt.subplots(figsize=(10.0, 3.2))
    ax.plot(t, mu, color=C[0], lw=1.0)
    ax.fill_between(t, 0, mu, color=C[0], alpha=0.10)
    ax.set_xlabel("time  [s]"); ax.set_ylabel("manipulability")
    ax.set_ylim(0, mu.max() * 1.15)
    grid_on(ax, "y")
    ax.axhline(mu.min(), color=C[1], lw=1.2, ls=(0, (4, 2)))
    ax.text(t[-1], mu.min(), f"  worst {mu.min():.2e}", color=C[1], fontsize=8.2,
            va="center", ha="right", backgroundcolor=SURFACE)
    finish(fig, "06_manipulability.png", "Distance from a singularity along the path",
           "Yoshikawa measure on the five-row task Jacobian. It never reaches zero, so the "
           "arm is not asked to pass through a singular configuration.")


# --- 7. gravity torque -------------------------------------------------------

def fig_gravity():
    G = D["path_grav"]
    t = D["path_t"][::10][:len(G)]
    fig, ax = plt.subplots(figsize=(10.0, 3.6))
    for i in range(5):
        ax.plot(t, G[:, i], color=C[i], lw=1.3, label=JOINTS[i])
    ax.set_xlabel("time  [s]"); ax.set_ylabel("gravity torque  [N m]")
    grid_on(ax, "y")
    ax.legend(ncol=5, fontsize=8.2, loc="upper center", bbox_to_anchor=(0.5, 1.16))
    peak = np.abs(G).max()
    ax.text(0.995, 0.06, f"peak {peak:.2f} N m against a 20 N m limit",
            transform=ax.transAxes, ha="right", color=INK2, fontsize=8.4)
    finish(fig, "07_gravity.png", "Gravity load the axes carry while working",
           "Recursive Newton-Euler with the description's own inertias. joint_1 is vertical, "
           "so gravity never loads it.")


# --- 8. step response --------------------------------------------------------

def fig_step():
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 3.8), sharey=True)
    for ax, tag, label in [(axes[0], "noff", "PID alone"),
                           (axes[1], "ff", "PID with gravity feedforward")]:
        T, Q, QR = D[f"step_T_{tag}"], D[f"step_Q_{tag}"], D[f"step_QR_{tag}"]
        ax.plot(T, np.rad2deg(QR[:, 1] - QR[0, 1]), color=MUTED, lw=1.2,
                ls=(0, (4, 2)), label="reference")
        for i in range(5):
            ax.plot(T, np.rad2deg(Q[:, i] - Q[0, i]), color=C[i], lw=1.3, label=JOINTS[i])
        ax.set_xlabel("time  [s]"); ax.set_title(label, fontsize=10, loc="left", pad=8)
        grid_on(ax, "y")
    axes[0].set_ylabel("displacement  [deg]")
    axes[1].legend(ncol=2, fontsize=8, loc="lower right")
    finish(fig, "08_step.png", "Five degree step, every axis at once",
           "Gains are critically damped by construction. Without the gravity term the loaded "
           "axes settle to an offset the integrator has to walk out.")


# --- 9. tracking -------------------------------------------------------------

def _spans(t, flag):
    """Contiguous [start, end] time spans where `flag` is true."""
    out, i, n = [], 0, len(flag)
    while i < n:
        if flag[i]:
            j = i
            while j + 1 < n and flag[j + 1]:
                j += 1
            out.append((t[i], t[min(j + 1, n - 1)]))
            i = j + 1
        else:
            i += 1
    return out


def fig_tracking():
    T, err = D["trk_cart_T"], D["trk_cart_err"] * 1e6
    down = D["trk_cart_down"].astype(bool)
    settled = D["trk_cart_settled"].astype(bool)
    TAU = D["trk_TAU"]; Tt = D["trk_T"]
    fig, axes = plt.subplots(2, 1, figsize=(10.0, 5.2), sharex=True,
                             gridspec_kw={"height_ratios": [1.25, 1]})
    # Shade where the needle is clear of the work: the error there is real but
    # it is not on the drawing, and at ten times the feed it dwarfs everything
    # that is.
    for a, b in _spans(T, ~down):
        axes[0].axvspan(a, b, color=MUTED, alpha=0.10, lw=0, zorder=0)
    axes[0].plot(T, np.where(down, err, np.nan), color=C[0], lw=1.2, zorder=3)
    axes[0].plot(T, np.where(down, np.nan, err), color=MUTED, lw=1.0, zorder=2)
    axes[0].axhline(300, color=C[1], lw=1.2, ls=(0, (4, 2)), zorder=4)
    axes[0].text(T[-1], 320, "  a 0.3 mm line width", color=C[1], fontsize=8.2,
                 ha="right", va="bottom")
    axes[0].text(0.012, 0.93, "in the work", transform=axes[0].transAxes,
                 color=C[0], fontsize=8.2, fontweight="bold", va="top")
    axes[0].text(0.012, 0.84, "travel, needle clear", transform=axes[0].transAxes,
                 color=MUTED, fontsize=8.2, fontweight="bold", va="top")
    for a, b in _spans(T, down & ~settled):
        axes[0].axvspan(a, b, color=C[3], alpha=0.16, lw=0, zorder=1)
    axes[0].text(0.012, 0.75, "settling after a plunge", transform=axes[0].transAxes,
                 color=C[3], fontsize=8.2, fontweight="bold", va="top")
    axes[0].set_ylabel("tip error  [um]")
    grid_on(axes[0], "y")
    for i in range(5):
        axes[1].plot(Tt, TAU[:, i], color=C[i], lw=1.0, label=JOINTS[i])
    axes[1].set_ylabel("commanded torque  [N m]"); axes[1].set_xlabel("time  [s]")
    axes[1].legend(ncol=5, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, 1.22))
    grid_on(axes[1], "y")
    finish(fig, "09_tracking.png", "Following the logo with PID and gravity feedforward",
           f"Once settled the tip holds {err[settled].mean():.0f} um against a 300 um line. "
           f"Travel sits at {err[~down].mean():.0f} um because the velocity lag scales with feed, "
           f"and the needle enters the work still {err[down].max():.0f} um out because nothing "
           "waits for the plunge to settle. Only the settled figure is about the drawing.")


# --- 10. control study -------------------------------------------------------

def fig_control_study():
    import json
    f = DATA / "control_study.json"
    if not f.exists():
        print("   (skipping 10, control_study.json not found)")
        return
    S = json.loads(f.read_text())
    modes = list(S["modes"].keys())
    mean = [S["modes"][m]["marking_settled_mean_um"] for m in modes]
    mx = [S["modes"][m]["marking_max_um"] for m in modes]

    fig, axes = plt.subplots(1, 2, figsize=(10.6, 3.9))
    y = np.arange(len(modes))[::-1]
    axes[0].barh(y + 0.18, mean, height=0.33, color=C[0], label="settled, marking")
    axes[0].barh(y - 0.18, mx, height=0.33, color=C[2], label="worst, marking")
    axes[0].set_yticks(y); axes[0].set_yticklabels(modes)
    axes[0].set_xscale("log"); axes[0].set_xlabel("tip error  [um], log scale")
    axes[0].axvline(300, color=C[1], lw=1.2, ls=(0, (4, 2)))
    axes[0].text(300, -0.62, " 0.3 mm line", color=C[1], fontsize=8.2,
                 ha="left", va="center")
    axes[0].legend(fontsize=8.2, loc="lower right")
    for yy, v in zip(y + 0.18, mean):
        axes[0].text(v * 1.15, yy, f"{v:,.0f}", va="center", fontsize=7.8, color=INK2)
    grid_on(axes[0], "x")

    wns = sorted(float(k) for k in S["sweep"])
    ok = [w for w in wns if not S["sweep"][str(w)]["diverged"]]
    sm = [S["sweep"][str(w)]["marking_settled_mean_um"] for w in ok]
    gone = [w for w in wns if S["sweep"][str(w)]["diverged"]]
    axes[1].plot(ok, sm, "-o", color=C[0], ms=5, mec=SURFACE, mew=1.2)
    for w in gone:
        axes[1].axvline(w, color=C[7], lw=1.0, ls=(0, (2, 2)))
    if gone:
        axes[1].axvspan(min(gone), max(wns) * 1.05, color=C[7], alpha=0.07, lw=0)
        axes[1].text(min(gone), max(sm), "  diverges", color=C[7], fontsize=8.2,
                     ha="left", va="top")
    axes[1].text(max(wns), 300, "0.3 mm line ", color=C[1], fontsize=8.2,
                 ha="right", va="bottom")
    axes[1].axhline(300, color=C[1], lw=1.2, ls=(0, (4, 2)))
    axes[1].set_xlabel("closed loop bandwidth  $\\omega_n$  [rad/s]")
    axes[1].set_ylabel("mean tip error  [um]")
    axes[1].set_yscale("log")
    for w, v in zip(ok, sm):
        axes[1].annotate(f"{v:,.0f}", (w, v), textcoords="offset points",
                         xytext=(0, 8), fontsize=7.8, color=INK2, ha="center")
    grid_on(axes[1], "y")
    axes[1].set_title("raising the gain instead", fontsize=10, loc="left", pad=8)
    axes[0].set_title("feeding the reference forward", fontsize=10, loc="left", pad=8)
    finish(fig, "10_control_study.png", "What actually removes the tracking error",
           f"Over the busiest {S['window_s']:.0f} s of the logo: {S['window_entries']} needle entries inside "
           "the O. Feeding the reference forward halves the settled error and lowers peak torque; "
           "raising the bandwidth to 40 rad/s divides it by fifteen, and above that the loop "
           "diverges at 200 Hz. Neither touches the worst bars, which are the entries themselves.")


# --- 11. contour tracing, retired method against the traced one --------------

def _angle_sorted(blob, xs, ys):
    """The method this replaced: edge pixels sorted by angle about the centroid.

    Kept here only so the figure can show what it did. It is a correct outline
    exactly when the region is convex and has no hole, which a disc is and a
    letterform is not.
    """
    from scipy import ndimage
    edge = blob & ~ndimage.binary_erosion(blob)
    r, c = np.nonzero(edge)
    P = np.column_stack([xs[c], ys[r]])
    ctr = P.mean(axis=0)
    P = P[np.argsort(np.arctan2(P[:, 1] - ctr[1], P[:, 0] - ctr[0]))]
    return np.vstack([P, P[:1]])


def fig_contours():
    from scipy import ndimage
    shapes = [("disc", "convex, no hole"), ("C", "concave"),
              ("B", "two enclosed holes")]
    fig, axes = plt.subplots(2, 3, figsize=(10.2, 6.8))

    for j, (kind, note) in enumerate(shapes):
        mask, (xs, ys) = rp._shape(kind)
        lab, _ = ndimage.label(mask, structure=np.ones((3, 3)))
        blob = lab == 1
        ext = [xs[0], xs[-1], ys[-1], ys[0]]

        for i, ax in enumerate(axes[:, j]):
            ax.imshow(blob, origin="upper", extent=ext, cmap="gray_r",
                      alpha=0.16, aspect="equal")
            if i == 0:
                ax.plot(*_angle_sorted(blob, xs, ys).T, color=C[7], lw=0.7)
            else:
                for n, seg in enumerate(rp._boundary_loops(blob, xs, ys)):
                    ax.plot(seg[:, 0], seg[:, 1], color=C[0] if n == 0 else C[2],
                            lw=1.4)
            ax.set_aspect("equal")
            ax.set_xticks([]); ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_visible(True); sp.set_color(GRID)
        axes[0, j].set_title(f"{kind}  —  {note}", fontsize=10, loc="left", pad=8)

    axes[0, 0].set_ylabel("sorted by angle\n(retired)", fontsize=9, color=INK2)
    axes[1, 0].set_ylabel("Moore traced\n(now)", fontsize=9, color=INK2)
    ax = axes[1, 2]
    ax.text(0.03, 0.97, "outline", transform=ax.transAxes, ha="left", va="top",
            fontsize=8.5, color=C[0], fontweight="bold")
    ax.text(0.03, 0.90, "hole", transform=ax.transAxes, ha="left", va="top",
            fontsize=8.5, color=C[2], fontweight="bold")

    finish(fig, "11_contours.png", "Contour tracing",
           "Angle about the centroid is right on a disc, collapses into a star on anything "
           "concave, and cannot see a hole. Moore tracing returns the outline plus one loop "
           "per hole, certified here at 100% boundary coverage.")


if __name__ == "__main__":
    print("figures:")
    fig_chain(); fig_workspace(); fig_panel(); fig_toolpath(); fig_joint_traj()
    fig_manip(); fig_gravity(); fig_step(); fig_tracking(); fig_control_study()
    fig_contours()
