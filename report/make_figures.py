"""
Figures of the report, generated from the versioned evaluation tables.

    python report/make_figures.py

Figure 1  execution success before and after the fixes, by tier and brick count
Figure 2  why an offset stack failed: the fingertips and the exposed studs (to scale)
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("svg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

ROOT = Path(__file__).resolve().parent.parent
OUT = Path(__file__).resolve().parent

# One hue, two steps: light = before, dark = after (validated as an ordinal ramp
# on a white surface: monotone lightness, light end 2.1:1 against the page).
BEFORE, AFTER = "#86b6ef", "#2a78d6"
INK, INK_SECONDARY, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
CRITICAL = "#d03b3b"

plt.rcParams.update({
    "font.family": ["Helvetica Neue", "Helvetica", "Arial", "sans-serif"],
    "font.size": 7.5,
    "svg.fonttype": "none",
    "axes.spines.top": False, "axes.spines.right": False, "axes.spines.left": False,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.6,
    "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
})


def read(name):
    with open(ROOT / "evaluation" / name / "results.csv") as f:
        return list(csv.DictReader(f))


def rate(rows, tier, n_bricks):
    group = [r for r in rows if r["tier"] == tier and r["n_bricks"] == n_bricks]
    return sum(r["success"] == "True" for r in group), len(group)


def rounded_column(ax, x, height, width, color, aspect):
    """
    Column with a rounded data end and a square foot on the baseline.
    aspect = (y data units per inch) / (x data units per inch), so that the
    corner radius, given in x units, comes out circular on the page.
    """
    if height <= 0:
        return
    radius = width * 0.22
    ax.add_patch(FancyBboxPatch((x - width / 2, 0), width, height,
                                boxstyle=f"round,pad=0,rounding_size={radius}",
                                mutation_aspect=aspect, linewidth=0, facecolor=color, zorder=3))
    ax.add_patch(Rectangle((x - width / 2, 0), width, min(height, radius * aspect),
                           linewidth=0, facecolor=color, zorder=3))


def figure_success():
    before, after = read("baseline_8dc8f92"), read("seed42")
    groups = [("1", "2", "Tier 1\n2 bricks"), ("2", "2", "Tier 2\n2 bricks"),
              ("2", "3", "Tier 2\n3 bricks"), ("2", "4", "Tier 2\n4 bricks")]

    fig, ax = plt.subplots(figsize=(3.35, 1.85))
    ax.set_xlim(-0.6, len(groups) - 0.4)
    ax.set_ylim(0, 118)
    fig.tight_layout(pad=0.3)
    fig.canvas.draw()
    box = ax.get_window_extent()
    aspect = (118 / box.height) / (len(groups) / box.width)

    width, gap = 0.30, 0.06
    for index, (tier, n_bricks, label) in enumerate(groups):
        for offset, rows, color, align in ((-(width + gap) / 2, before, BEFORE, "right"),
                                           ((width + gap) / 2, after, AFTER, "left")):
            ok, n = rate(rows, tier, n_bricks)
            value = 100 * ok / n
            rounded_column(ax, index + offset, value, width, color, aspect)
            # Labels grow outwards from the gap between the two columns, so two
            # equally tall columns do not print their values on top of each other.
            anchor = index + offset + (width / 2 if align == "right" else -width / 2)
            ax.text(anchor, value + 2.5, f"{ok}/{n}", ha=align, va="bottom", fontsize=6.2, color=INK_SECONDARY)
    ax.set_xticks(range(len(groups)), [g[2] for g in groups])
    ax.set_yticks([0, 25, 50, 75, 100], ["0", "25", "50", "75", "100 %"])
    ax.tick_params(length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    handles = [Rectangle((0, 0), 1, 1, color=BEFORE), Rectangle((0, 0), 1, 1, color=AFTER)]
    ax.legend(handles, ["before (commit 8dc8f92)", "after"], loc="upper center", bbox_to_anchor=(0.5, 1.13),
              ncol=2, frameon=False, fontsize=7, handlelength=1.0, handleheight=0.8, columnspacing=1.6)
    fig.tight_layout(pad=0.3)
    fig.savefig(OUT / "fig_success.svg")
    plt.close(fig)


def figure_collision():
    """Side view through an offset stack, to scale (millimetres)."""
    fig, axes = plt.subplots(1, 2, figsize=(3.35, 1.45), sharey=True)
    brick_h, stud_h, stud_r, pitch, wall = 19.0, 4.0, 4.2, 16.0, 1.0
    pad_w, tip = 8.0, 1.7                       # finger pad thickness, mesh tip below the pad
    for ax, (title, tip_above_bottom) in zip(axes, (("before: grasp at the bottom", 0.0),
                                                     ("after: fingertips 9 mm higher", 8.8))):
        lower_x, upper_x = 0.0, 16.0           # upper brick offset by one stud
        top = brick_h
        # lower brick and its two studs
        ax.add_patch(Rectangle((lower_x - 16, 0), 32, brick_h, facecolor="#e7e6e1", edgecolor=AXIS, linewidth=0.6))
        for sx in (-8, 8):
            ax.add_patch(Rectangle((lower_x + sx - stud_r, top), 2 * stud_r, stud_h,
                                   facecolor="#e7e6e1", edgecolor=AXIS, linewidth=0.6))
        # upper brick (hollow underside: drawn above the studs it sits on)
        seat = top + 0.2
        ax.add_patch(Rectangle((upper_x - 16, seat), 32, brick_h, facecolor="#cde2fb", edgecolor="#86b6ef", linewidth=0.6))
        ax.add_patch(Rectangle((upper_x - 15, seat), 30, stud_h, facecolor="white", edgecolor="none"))
        ax.add_patch(Rectangle((lower_x + 8 - stud_r, top), 2 * stud_r, stud_h,
                               facecolor="#e7e6e1", edgecolor=AXIS, linewidth=0.6))
        # fingers
        finger_bottom = seat + tip_above_bottom
        for side in (-1, 1):
            x0 = upper_x + side * 16 if side > 0 else upper_x - 16 - pad_w
            ax.add_patch(Rectangle((x0, finger_bottom), pad_w, 30, facecolor=INK_SECONDARY, edgecolor="none", alpha=0.85))
        # exposed stud under the left finger
        stud_top = top + stud_h
        if finger_bottom < stud_top:
            # only the part of the stud that lies inside the finger
            finger_left = upper_x - 16 - pad_w
            ax.add_patch(Rectangle((finger_left, finger_bottom), -8 + stud_r - finger_left, stud_top - finger_bottom,
                                   facecolor=CRITICAL, edgecolor="none", zorder=5))
            ax.annotate("finger hits\nexposed stud", xy=(-6, stud_top - 1.5), xytext=(-27, 40),
                        fontsize=6.5, color=INK, ha="center",
                        arrowprops=dict(arrowstyle="-", color=CRITICAL, linewidth=0.8))
        else:
            ax.annotate(f"{finger_bottom - stud_top:.0f} mm clear", xy=(-8, stud_top + 0.5), xytext=(-27, 40),
                        fontsize=6.5, color=INK, ha="center",
                        arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8))
        ax.set_title(title, fontsize=7, color=INK_SECONDARY, pad=3)
        ax.set_xlim(-40, 46); ax.set_ylim(-1, 52)
        ax.set_aspect("equal"); ax.axis("off")
    axes[0].text(0, 9.5, "support", ha="center", va="center", fontsize=6.5, color=INK_SECONDARY)
    axes[0].text(16, 33, "brick", ha="center", va="center", fontsize=6.5, color=INK_SECONDARY)
    fig.tight_layout(pad=0.2, w_pad=0.4)
    fig.savefig(OUT / "fig_collision.svg")
    plt.close(fig)


if __name__ == "__main__":
    figure_collision()
    figure_success()
    print("figures written to", OUT)
