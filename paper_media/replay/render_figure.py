"""Static vector timeline figure (PDF and SVG) for the paper.

Top to bottom on one session time axis: a filmstrip of camera frames at key
events, the four swimlanes, and the signal strip with threshold lines. Sized
for an IEEE page: 7.16 in (two column) or 3.5 in (single column). Every text
element is at least 7 pt.
"""
from __future__ import annotations

import textwrap
from pathlib import Path

from . import style as S
from .timeline import Prepared

MIN_PT = 7.0
# Labelled in the figure. Everything else is a marker only; a whole run has
# hundreds of events and a printed page cannot carry a label for each.
LABEL_KINDS = {"fire", "verdict", "run_verdict", "halt", "timeout", "look"}
LABEL_KINDS_SHORT_WINDOW = LABEL_KINDS | {"step_start", "end_step", "arm"}
SHORT_WINDOW_S = 180.0
BIG_KINDS = LABEL_KINDS_SHORT_WINDOW | {"wake", "look_frame", "step_claim"}
LANE_NAMES = {"executor": "Executor Agent", "bus": "ROS 2 Event Bus",
              "monitors": "Deterministic Monitors", "watchdog": "WatchDog Agent"}


def choose_frame_times(prep: Prepared, n_max: int) -> list[float]:
    """Session times for the filmstrip. Only times that have a camera frame are returned."""
    ov = prep.story.figure_frames_at
    if ov:
        return [float(x) for x in ov][:n_max]
    r0, r1 = prep.timemap.r_start, prep.timemap.r_end
    if prep.frames_are_evidence:
        # Sparse stills: key-event times rarely have one, so use the stills' own times.
        ts = [prep.session.t(e) for e in prep.frames.epochs if r0 <= prep.session.t(e) <= r1]
        if len(ts) <= n_max:
            return ts
        step = (len(ts) - 1) / (n_max - 1)
        return [ts[round(i * step)] for i in range(n_max)]
    keys = sorted(prep.key_events, key=lambda e: e.t)
    # prefer the events a reader looks for first, then spread the rest
    pri = [e for e in keys if e.kind in ("fire", "halt") or (e.kind == "verdict" and e.label.startswith("FAIL"))]
    rest = [e for e in keys if e not in pri]
    chosen = pri[:n_max]
    if len(chosen) < n_max and rest:
        k = n_max - len(chosen)
        step = max(1, len(rest) // k)
        chosen += rest[::step][:k]
    times = sorted({round(e.t, 2) for e in chosen})
    out: list[float] = []
    span = (r1 - r0) or 1.0
    seen_paths: set[str] = set()
    for t in times:                     # drop near duplicates and times with no frame
        path = prep.frames.at(prep.session.epoch(t))
        if path is None or path in seen_paths:
            continue
        if not out or t - out[-1] > span / (n_max * 2):
            out.append(t)
            seen_paths.add(path)
    return out[:n_max]


def render_figure(prep: Prepared, out: Path, width: str = "double") -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import ConnectionPatch
    from PIL import Image

    S.apply_matplotlib_style()
    W = S.FIG_WIDTH_IN[width]
    single = width == "single"
    n_frames = 4 if single else 6
    has_sig = bool(prep.signals)
    keys_txt = [f"({i}) {ex['heading']}: {ex['text']}" if ex["heading"] else f"({i}) {ex['text']}"
                for i, ex in enumerate(prep.excerpts, 1)]
    key_block = "\n".join(textwrap.fill(k, 70 if single else 140) for k in keys_txt)
    key_lines = key_block.count("\n") + 1 if key_block else 0
    key_h_in = key_lines * MIN_PT * 1.25 / 72.0          # reserved height for the excerpt key
    ratios = [0.95 if single else 1.0, 1.7] + ([0.8] if has_sig else [])
    H = sum(ratios) + 0.45 + key_h_in
    fig = plt.figure(figsize=(W, H))
    gs = fig.add_gridspec(len(ratios), 1, height_ratios=ratios, hspace=0.14,
                          left=0.2 if single else 0.155, right=(0.86 if single else 0.93) if has_sig and len(prep.signals) > 1 else 0.985,
                          top=0.985, bottom=(0.35 + key_h_in) / H)
    ax_film = fig.add_subplot(gs[0])
    ax_sw = fig.add_subplot(gs[1])
    ax_sig = fig.add_subplot(gs[2], sharex=ax_sw) if has_sig else None
    r0, r1 = prep.timemap.r_start, prep.timemap.r_end
    span = r1 - r0

    # ---- swimlanes --------------------------------------------------------
    lanes = list(S.LANES)
    ypos = {ln: len(lanes) - 1 - i for i, ln in enumerate(lanes)}
    ax_sw.set_xlim(r0, r1)
    ax_sw.set_ylim(-0.7, len(lanes) - 0.3)
    ax_sw.set_yticks([ypos[ln] for ln in lanes])
    ax_sw.set_yticklabels([S.LANE_SHORT[ln] if single else LANE_NAMES[ln] for ln in lanes], fontsize=MIN_PT)
    for tick_label, ln in zip(ax_sw.get_yticklabels(), lanes):
        tick_label.set_color(S.LANE_COLOR[ln])
    for ln in lanes:
        ax_sw.axhline(ypos[ln], color=S.GRID, lw=0.5, zorder=0)
    ax_sw.spines["left"].set_visible(False)
    ax_sw.tick_params(axis="y", length=0)
    for a in prep.arrows:
        ax_sw.annotate("", xy=(a.dst.t, ypos[a.dst.lane]), xytext=(a.src.t, ypos[a.src.lane]),
                       arrowprops=dict(arrowstyle="-|>", color=a.color, lw=0.5, alpha=0.6, shrinkA=2, shrinkB=2,
                                       mutation_scale=5))
    label_kinds = LABEL_KINDS_SHORT_WINDOW if span <= SHORT_WINDOW_S else LABEL_KINDS
    room = span / (9 if single else 16)
    last_lab: dict[str, float] = {}
    flip: dict[str, int] = {}
    for e in prep.events:
        big = e.kind in BIG_KINDS
        ax_sw.plot([e.t], [ypos[e.lane]], marker="o", ms=3.0 if big else 1.2,
                   color=S.event_color(e.lane, e.kind, e.label), lw=0, zorder=3)
        if e.kind in label_kinds and e.t - last_lab.get(e.lane, -1e9) > room:
            k = flip.get(e.lane, 0)
            dy = 0.24 if k % 2 == 0 else -0.30
            ax_sw.text(e.t, ypos[e.lane] + dy, textwrap.shorten(e.label, 18 if single else 24, placeholder="..."),
                       fontsize=MIN_PT, ha="left", va="center", clip_on=True,
                       color=S.event_color(e.lane, e.kind, e.label), zorder=4)
            last_lab[e.lane] = e.t
            flip[e.lane] = k + 1
    # Prompt excerpts: numbered markers on their events, text in a key below the axis,
    # so a callout never covers an event.
    for i, ex in enumerate(prep.excerpts, 1):
        lane = ex["lane"] if ex["lane"] in ypos else "watchdog"
        ax_sw.annotate(str(i), xy=(ex["t"], ypos[lane]), xytext=(0, 9), textcoords="offset points",
                       fontsize=MIN_PT, ha="center", va="bottom", color=S.INK, zorder=6,
                       bbox=dict(boxstyle="circle,pad=0.15", fc=S.HIGHLIGHT_BG, ec=S.LANE_COLOR[lane], lw=0.5))
    if ax_sig is None:
        ax_sw.set_xlabel("session time [s]", fontsize=MIN_PT)
    else:
        ax_sw.tick_params(labelbottom=False)

    # ---- signals: one y axis per unit ---------------------------------------
    if ax_sig is not None:
        colors = [S.EXECUTOR_BLUE, S.WATCHDOG_GREEN, S.BUS_GRAY]
        axes = [ax_sig]
        units = []
        for i, sig in enumerate(list(prep.signals.values())[:2]):
            ax = ax_sig if i == 0 else ax_sig.twinx()
            if i:
                axes.append(ax)
                ax.spines["right"].set_visible(True)
            ts = [t for t in sig.t if r0 <= t <= r1]
            vs = [v for t, v in zip(sig.t, sig.v) if r0 <= t <= r1]
            ax.plot(ts, vs, color=colors[i], lw=0.7)
            ax.set_ylabel(f"{sig.name} [{sig.unit}]", fontsize=MIN_PT, color=colors[i])
            ax.tick_params(axis="y", colors=colors[i], labelsize=MIN_PT)
            units.append(sig.unit)
            for th in prep.thresholds:
                if th["signal"] == sig.name:
                    ax.axhline(th["value"], color=S.FAIL_RED, lw=0.6, ls="--")
                    ax.text(r0 + 0.005 * span, th["value"], th["label"], fontsize=MIN_PT, color=S.FAIL_RED,
                            ha="left", va="bottom")
        ax_sig.set_xlabel("session time [s]", fontsize=MIN_PT)
        ax_sig.set_xlim(r0, r1)

    # ---- filmstrip --------------------------------------------------------
    ax_film.axis("off")
    times = choose_frame_times(prep, n_frames)
    if times:
        gap = 0.012
        wslot = (1 - gap * (n_frames - 1)) / n_frames
        x_start = (1 - (len(times) * wslot + (len(times) - 1) * gap)) / 2   # centre a short strip
        for i, t in enumerate(times):
            path = prep.frames.at(prep.session.epoch(t))
            x = x_start + i * (wslot + gap)
            sub = ax_film.inset_axes([x, 0.18, wslot, 0.82])
            sub.set_xticks([])
            sub.set_yticks([])
            if path:
                sub.imshow(Image.open(path).convert("RGB"))
            else:
                sub.text(0.5, 0.5, "no frame", fontsize=MIN_PT, ha="center", va="center", transform=sub.transAxes)
            for sp in sub.spines.values():
                sp.set_linewidth(0.5)
            ax_film.text(x + wslot / 2, 0.0, f"t = {t:.1f} s", fontsize=MIN_PT, ha="center", va="bottom",
                         transform=ax_film.transAxes)
            fig.add_artist(ConnectionPatch(xyA=(0.5, 0.0), coordsA=sub.transAxes, xyB=(t, len(lanes) - 0.3),
                                           coordsB=ax_sw.transData, color=S.MUTED, lw=0.4, ls=":"))
    else:
        ax_film.text(0.5, 0.5, "no camera frames in the window", fontsize=MIN_PT, ha="center", va="center")

    if key_block:
        fig.text(0.01, 0.04 / H, key_block, fontsize=MIN_PT, ha="left", va="bottom", color=S.INK,
                 linespacing=1.15)

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    pdf = out.with_suffix(".pdf")
    svg = out.with_suffix(".svg")
    fig.savefig(pdf)
    fig.savefig(svg)
    plt.close(fig)
    print(f"wrote {pdf} and {svg} ({W} in wide{', frames are evidence stills' if prep.frames_are_evidence else ''})")
    return [pdf, svg]
