"""Task hand-over figure: one task specification, two agents, two different briefs.

Sits between the architecture figure (who the agents are) and the data flow figure
(what they do during a run). Draws boxes and arrows with matplotlib; text stays
text in the SVG so it can be edited in the layout tool.

    ~/.venvs/paper_media/bin/python -m paper_media.figures.task_handover [--out DIR]

Writes task_handover_double.svg (7.16 in), task_handover.png (300 dpi) and the
text of every box as task_handover.txt for the caption writer.
"""
from __future__ import annotations

import argparse
import textwrap
from pathlib import Path

from paper_media.replay import style as S
from paper_media.replay.icons import draw_mpl

OUT_DEFAULT = Path(__file__).resolve().parent / "out"

# Box text. Wording paraphrases agents/tasks/fmb_assembly_2.{executor,watchdog}.yaml.
TASK = ("Task specification",
        ["Assemble the FMB board: seat u_brown,",
         "u_orange, line_green and inverted_u_yellow",
         "on the base1 fixture, in that order.",
         "",
         "Requirements: each part seated in tolerance,",
         "gripped securely, clamp effort within its limit,",
         "every other part left undisturbed."])
REGISTRY = ("Part registry",
            ["Per part: position and orientation tolerance,",
             "lift height, slip and disturbance tolerance,",
             "maximum clamp effort. Source of every threshold."])
EXECUTOR = ("Executor Agent", "does the work",
            ["Receives HOW: the step plan.",
             "12 steps: pick, approach, seat, for each part;",
             "grasp widths, approach offsets, placement frames.",
             "",
             "Tools: move the arm, open and close the gripper,",
             "read poses. Announces each step, reports its outcome."])
WATCHDOG = ("WatchDog Agent", "checks the work",
            ["Receives WHAT must be true, never the executor's path.",
             "At each place step: part seated at its frame, grasped at",
             "the right width and lifted clear, base1 unmoved.",
             "Throughout: clamp effort in limit, no slip, bystanders still.",
             "Watch the work area; report anything unexpected.",
             "",
             "Tools: observe, arm monitors, record verdicts.",
             "Cannot move the robot."])
RUN_ICONS = ["megaphone", "shield", "gate", "arm", "bolt", "gavel"]
RUN = ("The run",
       ["Executor announces a step",
        "WatchDog arms monitors",
        "step gate released",
        "robot moves, monitors watch",
        "fire → WatchDog judges",
        "verdict per fire and per run"])


TINT = {"ink": "#f1f3f5", "blue": "#e8f0fa", "green": "#e6f5ec", "amber": "#fdf3e3", "gray": "#f6f8fa"}


def _box(ax, x, y, w, h, color, fill, lw=0.9, ls="-", shadow=True):
    from matplotlib.patches import FancyBboxPatch
    if shadow:
        ax.add_patch(FancyBboxPatch((x + 0.03, y - 0.03), w, h, boxstyle="round,pad=0,rounding_size=0.14",
                                    linewidth=0, facecolor="#000000", alpha=0.08))
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.14",
                                linewidth=lw, edgecolor=color, facecolor=fill, linestyle=ls))


def _header(ax, x, y_top, w, color, title, tag="", icon=None):
    """Filled title bar across the top of a box, white text; optional right-aligned tag."""
    from matplotlib.patches import FancyBboxPatch
    hh = 0.30
    ax.add_patch(FancyBboxPatch((x, y_top - hh), w, hh, boxstyle="round,pad=0,rounding_size=0.14",
                                linewidth=0, facecolor=color))
    ax.add_patch(FancyBboxPatch((x, y_top - hh), w, hh / 2, boxstyle="square,pad=0", linewidth=0, facecolor=color))
    tx = x + 0.11
    if icon:
        draw_mpl(ax, icon, x + 0.11, y_top - hh + 0.055, 0.19, "white", lw=0.8)
        tx += 0.28
    ax.text(tx, y_top - hh / 2, title, fontsize=8, color="white", fontweight="bold", va="center", ha="left")
    if tag:
        ax.text(x + w - 0.11, y_top - hh / 2, tag, fontsize=5.8, color="white", va="center", ha="right", style="italic")
    return y_top - hh - 0.09


def _text(ax, x, y, lines, size, color=S.INK, weight="normal", lh=None):
    lh = lh or size * 0.0155
    for i, ln in enumerate(lines):
        bold = ln.startswith("**")
        ax.text(x, y - i * lh, ln.strip("*"), fontsize=size, color=color, fontweight="bold" if bold else weight,
                va="top", ha="left")


def _arrow(ax, p0, p1, color, text="", lw=1.4):
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, shrinkA=0, shrinkB=0,
                                                       mutation_scale=10))
    if text:
        mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        ax.text(mx, my, text, fontsize=5.8, color=color, ha="center", va="center", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.28", fc="white", ec=color, lw=0.7))


def _pill(ax, x, y, w, h, color, n, text, fill="white", icon=None):
    from matplotlib.patches import FancyBboxPatch, Circle
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.09",
                                linewidth=0.8, edgecolor=color, facecolor=fill))
    ax.add_patch(Circle((x + 0.14, y + h / 2), 0.085, facecolor=color, edgecolor="none"))
    ax.text(x + 0.14, y + h / 2, str(n), fontsize=5.6, color="white", fontweight="bold", ha="center", va="center")
    if icon:
        draw_mpl(ax, icon, x + w - 0.26, y + 0.045, h - 0.09, color, lw=0.7)
    ax.text(x + 0.31, y + h / 2, text, fontsize=5.6, color=color if color == S.FAIL_RED else S.INK,
            fontweight="bold" if color == S.FAIL_RED else "normal", ha="left", va="center")


def render(out_dir: Path) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    S.apply_matplotlib_style()
    matplotlib.rcParams["svg.fonttype"] = "none"      # editable text in the SVG
    W, H = S.FIG_WIDTH_IN["double"], 3.05
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    top = 2.92

    # column 1: task + registry
    x1, w1 = 0.10, 1.92
    _box(ax, x1, 1.50, w1, top - 1.50, S.INK, TINT["ink"])
    y = _header(ax, x1, top, w1, S.INK, TASK[0], "shared", icon="clipboard")
    _text(ax, x1 + 0.10, y, TASK[1], 5.9)
    _box(ax, x1, 0.14, w1, 1.08, S.GT_AMBER, TINT["amber"])
    y = _header(ax, x1, 1.22, w1, S.GT_AMBER, REGISTRY[0], "per part", icon="ruler")
    _text(ax, x1 + 0.10, y, REGISTRY[1], 5.9)

    # column 2: the two agents
    x2, w2 = 2.62, 2.42
    _box(ax, x2, 1.66, w2, top - 1.66, S.EXECUTOR_BLUE, TINT["blue"], lw=1.1)
    y = _header(ax, x2, top, w2, S.EXECUTOR_BLUE, EXECUTOR[0], EXECUTOR[1], icon="arm")
    _text(ax, x2 + 0.10, y, EXECUTOR[2], 5.9)
    _box(ax, x2, 0.14, w2, 1.38, S.WATCHDOG_GREEN, TINT["green"], lw=1.1)
    y = _header(ax, x2, 1.52, w2, S.WATCHDOG_GREEN, WATCHDOG[0], WATCHDOG[1], icon="eye")
    _text(ax, x2 + 0.10, y, WATCHDOG[2], 5.9)

    # column 3: the run, numbered pills
    x3, w3 = 5.28, 1.78
    _box(ax, x3, 0.14, w3, top - 0.14, S.BUS_GRAY, TINT["gray"], ls=(0, (3, 2)), shadow=False)
    y = _header(ax, x3, top, w3, S.BUS_GRAY, RUN[0], "every step", icon="play")
    ph, gap = 0.27, 0.10
    for i, ln in enumerate(RUN[1]):
        col = S.FAIL_RED if ln.startswith("fire") else (
            S.EXECUTOR_BLUE if ln.startswith("Executor") or ln.startswith("robot") else
            S.WATCHDOG_GREEN if ln.startswith("WatchDog") or ln.startswith("verdict") else S.BUS_GRAY)
        _pill(ax, x3 + 0.10, y - ph, w3 - 0.20, ph, col, i + 1, ln, icon=RUN_ICONS[i])
        y -= ph + gap
    ax.text(x3 + w3 / 2, 0.22, "continues in: How it works", fontsize=5.6, color=S.MUTED, ha="center", va="bottom",
            style="italic")

    # arrows: task -> both agents, registry -> both, agents -> run
    xm = x1 + w1
    _arrow(ax, (xm, 2.30), (x2, 2.30), S.EXECUTOR_BLUE, "HOW")
    _arrow(ax, (xm, 1.62), (x2, 1.00), S.WATCHDOG_GREEN, "WHAT")
    _arrow(ax, (xm, 0.50), (x2, 0.40), S.GT_AMBER, "limits")
    _arrow(ax, (xm, 0.95), (x2, 2.00), S.GT_AMBER)
    _arrow(ax, (x2 + w2, 2.30), (x3, 2.30), S.EXECUTOR_BLUE)
    _arrow(ax, (x2 + w2, 0.75), (x3, 0.75), S.WATCHDOG_GREEN)
    ax.text(x2 + w2 / 2, 1.59, "no channel between the agents except the event stream",
            fontsize=5.5, color=S.MUTED, ha="center", va="center", style="italic",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=S.GRID, lw=0.5))

    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, kw in (("task_handover_double.svg", {}), ("task_handover.png", {"dpi": 300})):
        p = out_dir / name
        fig.savefig(p, **kw)
        paths.append(p)
    plt.close(fig)
    txt = out_dir / "task_handover.txt"
    txt.write_text("\n\n".join(textwrap.dedent("\n".join([t[0]] + list(t[-1]))) for t in (TASK, REGISTRY, EXECUTOR, WATCHDOG, RUN)) + "\n",
                   encoding="utf-8")
    paths.append(txt)
    return paths


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    a = ap.parse_args()
    for p in render(a.out):
        print("wrote", p)


if __name__ == "__main__":
    main()
