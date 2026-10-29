"""Static vector timeline figure (PDF and SVG) for the paper.

Top to bottom on one session time axis: a filmstrip of camera frames at key
events, the four swimlanes, and the signal strip with threshold lines. Sized
for an IEEE page: 7.16 in (two column) or 3.5 in (single column). Every text
element is at least 7 pt.
"""
from __future__ import annotations

import textwrap
from pathlib import Path
from typing import Optional

from . import style as S
from .timeline import Prepared

MIN_PT = 7.0
LABEL_KINDS = {"step_start", "end_step", "arm", "fire", "verdict", "run_verdict", "look", "halt", "timeout"}


def choose_frame_times(prep: Prepared, n_max: int) -> list[float]:
    ov = prep.story.figure_frames_at
    if ov:
        return [float(x) for x in ov][:n_max]
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
    span = (prep.timemap.r_end - prep.timemap.r_start) or 1.0
    for t in times:                     # drop near duplicates so thumbnails do not overlap
        if not out or t - out[-1] > span / (n_max * 2):
            out.append(t)
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
    rows = [1.05 if not single else 0.85, 1.75, 0.75 if prep.signals else 0.0]
    H = sum(rows) + 0.35
    fig = plt.figure(figsize=(W, H))
    gs = fig.add_gridspec(3 if prep.signals else 2, 1, height_ratios=[r for r in rows if r > 0], hspace=0.12,
                          left=0.17 if single else 0.12, right=0.985, top=0.985, bottom=0.09)
    ax_film = fig.add_subplot(gs[0])
    ax_sw = fig.add_subplot(gs[1])
    ax_sig = fig.add_subplot(gs[2], sharex=ax_sw) if prep.signals else None
    r0, r1 = prep.timemap.r_start, prep.timemap.r_end

    # ---- swimlanes --------------------------------------------------------
    lanes = list(S.LANES)
    ypos = {ln: len(lanes) - 1 - i for i, ln in enumerate(lanes)}
    ax_sw.set_xlim(r0, r1)
    ax_sw.set_ylim(-0.6, len(lanes) - 0.4)
    ax_sw.set_yticks([ypos[ln] for ln in lanes])
    names = {ln: (S.LANE_SHORT[ln] if single else S.LANE_TITLES[ln].replace(" (/watchdog/events)", "")) for ln in lanes}
    ax_sw.set_yticklabels([names[ln] for ln in lanes], fontsize=MIN_PT)
    for tick_label, ln in zip(ax_sw.get_yticklabels(), lanes):
        tick_label.set_color(S.LANE_COLOR[ln])
    for ln in lanes:
        ax_sw.axhline(ypos[ln], color=S.GRID, lw=0.5, zorder=0)
    ax_sw.spines["left"].set_visible(False)
    ax_sw.tick_params(axis="y", length=0)
    for a in prep.arrows:
        ax_sw.annotate("", xy=(a.dst.t, ypos[a.dst.lane]), xytext=(a.src.t, ypos[a.src.lane]),
                       arrowprops=dict(arrowstyle="-|>", color=a.color, lw=0.6, alpha=0.7, shrinkA=2, shrinkB=2))
    label_room = (r1 - r0) / (14 if single else 26)
    last_lab: dict[str, float] = {}
    flip: dict[str, int] = {}
    for e in prep.events:
        big = e.kind in LABEL_KINDS or e.kind in ("wake", "look_frame", "step_claim")
        ax_sw.plot([e.t], [ypos[e.lane]], marker="o", ms=3.2 if big else 1.4,
                   color=S.event_color(e.lane, e.kind, e.label), lw=0, zorder=3)
        if e.kind in LABEL_KINDS and e.t - last_lab.get(e.lane, -1e9) > label_room:
            k = flip.get(e.lane, 0)
            dy = 0.22 if k % 2 == 0 else -0.30
            ax_sw.text(e.t, ypos[e.lane] + dy, textwrap.shorten(e.label, 22, placeholder="..."), fontsize=MIN_PT,
                       ha="left", va="center", color=S.event_color(e.lane, e.kind, e.label), zorder=4)
            last_lab[e.lane] = e.t
            flip[e.lane] = k + 1
    for ex in prep.excerpts:
        lane = ex["lane"] if ex["lane"] in ypos else "watchdog"
        txt = f"{ex['heading']}: {ex['text']}" if ex["heading"] else ex["text"]
        txt = "\n".join(textwrap.wrap(txt, 34 if single else 46)[:3])
        ax_sw.annotate(txt, xy=(ex["t"], ypos[lane]), xytext=(0, -26 if ypos[lane] > 0 else 20),
                       textcoords="offset points", fontsize=MIN_PT, ha="left", va="top",
                       bbox=dict(boxstyle="round,pad=0.25", fc=S.HIGHLIGHT_BG, ec=S.LANE_COLOR.get(lane, S.INK), lw=0.5),
                       arrowprops=dict(arrowstyle="-", color=S.LANE_COLOR.get(lane, S.INK), lw=0.5), zorder=5)
    if ax_sig is None:
        ax_sw.set_xlabel("session time [s]", fontsize=MIN_PT)
    else:
        ax_sw.tick_params(labelbottom=False)

    # ---- signals ----------------------------------------------------------
    if ax_sig is not None:
        colors = [S.EXECUTOR_BLUE, S.WATCHDOG_GREEN, S.BUS_GRAY]
        for i, sig in enumerate(list(prep.signals.values())[:3]):
            ts = [t for t in sig.t if r0 <= t <= r1]
            vs = [v for t, v in zip(sig.t, sig.v) if r0 <= t <= r1]
            ax_sig.plot(ts, vs, color=colors[i % 3], lw=0.8, label=f"{sig.name} [{sig.unit}]")
            for th in prep.thresholds:
                if th["signal"] == sig.name:
                    ax_sig.axhline(th["value"], color=S.FAIL_RED, lw=0.7, ls="--")
                    ax_sig.text(r1, th["value"], f" {th['label']}", fontsize=MIN_PT, color=S.FAIL_RED,
                                ha="right", va="bottom")
        ax_sig.legend(loc="upper left", fontsize=MIN_PT, frameon=False, handlelength=1.2)
        ax_sig.set_xlabel("session time [s]", fontsize=MIN_PT)
        ax_sig.set_xlim(r0, r1)

    # ---- filmstrip --------------------------------------------------------
    ax_film.axis("off")
    times = choose_frame_times(prep, n_frames)
    if times:
        n = len(times)
        gap = 0.012
        wslot = (1 - gap * (n - 1)) / n
        for i, t in enumerate(times):
            path = prep.frames.at(prep.session.epoch(t))
            sub = ax_film.inset_axes([i * (wslot + gap), 0.16, wslot, 0.84])
            sub.set_xticks([])
            sub.set_yticks([])
            if path:
                sub.imshow(Image.open(path).convert("RGB"))
            else:
                sub.text(0.5, 0.5, "no frame", fontsize=MIN_PT, ha="center", va="center", transform=sub.transAxes)
            for sp in sub.spines.values():
                sp.set_linewidth(0.5)
            ax_film.text(i * (wslot + gap) + wslot / 2, 0.0, f"t = {t:.1f} s", fontsize=MIN_PT, ha="center",
                         va="bottom", transform=ax_film.transAxes)
            con = ConnectionPatch(xyA=(0.5, 0.0), coordsA=sub.transAxes, xyB=(t, len(lanes) - 0.4), coordsB=ax_sw.transData,
                                  color=S.MUTED, lw=0.4, ls=":")
            fig.add_artist(con)
    else:
        ax_film.text(0.5, 0.5, "no key events in the window", fontsize=MIN_PT, ha="center", va="center")

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    pdf = out.with_suffix(".pdf")
    svg = out.with_suffix(".svg")
    fig.savefig(pdf)
    fig.savefig(svg)
    plt.close(fig)
    print(f"wrote {pdf} and {svg} ({W} in wide{', frames are evidence stills' if prep.frames_are_evidence else ''})")
    return [pdf, svg]
