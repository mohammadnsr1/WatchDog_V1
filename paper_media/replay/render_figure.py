"""Static vector timeline figure (SVG, editable in PowerPoint) for the paper, plus its key frames.

The figure carries the swimlanes and, when the story selects any, the signal
strip with threshold lines. It has no camera images, no captions and no prompt
excerpts: those are added by hand in the layout tool. Each key frame time is
marked on the time axis with a small numbered tick, and export_frames() saves
the matching camera frames as separate, time-stamped PNGs so the two line up.

Sized for an IEEE page: 7.16 in (two column) or 3.5 in (single column). Every
text element is at least 7 pt.
"""
from __future__ import annotations

import csv
import shutil
from pathlib import Path

from . import style as S
from .timeline import Prepared

MIN_PT = 7.0
# Labelled in the figure. Everything else is a marker only; a whole run has
# hundreds of events and a printed page cannot carry a label for each.
LABEL_KINDS = {"fire", "verdict", "run_verdict", "halt", "timeout", "look", "observation"}
LABEL_KINDS_SHORT_WINDOW = LABEL_KINDS | {"step_start", "end_step", "arm", "gate_open", "wake", "step_claim"}
SHORT_WINDOW_S = 240.0
LABEL_PRIORITY = {"fire": 0, "halt": 0, "verdict": 1, "run_verdict": 1, "observation": 2, "timeout": 3,
                  "look": 4, "step_start": 5, "end_step": 5, "step_claim": 5, "gate_open": 6, "arm": 6, "wake": 7}
BIG_KINDS = LABEL_KINDS_SHORT_WINDOW | {"look_frame"}
LANE_NAMES = {"executor": "Executor Agent", "bus": "ROS 2 Event Bus",
              "monitors": "Deterministic Monitors", "watchdog": "WatchDog Agent"}
MAX_FRAME_GAP_S = 0.5
GROUP_GAP_FRAC = 0.06     # events of one kind closer than this fraction of the window form one group
MARKERS = {"begin_step": ("s", 3.2), "end_step": ("s", 3.2), "gate_open": ("^", 3.6),
           "arm": ("D", 3.0), "disarm": ("x", 3.0), "fire": ("o", 3.6),
           "look": ("*", 4.5), "observation": ("*", 4.5), "run_verdict": ("P", 4.0), "verdict": ("P", 4.0)}


def _short(label: str, n: int = 24) -> str:
    return label if len(label) <= n else label[:n - 3] + "..."


def group_labels(events, r0: float, r1: float, gap_s: float) -> list[dict]:
    """Grouped label items for the swimlanes.

    executor  one band per step, step_start to its step_claim (or the next step_start)
    monitors  runs of arm (or disarm) events closer than gap_s -> "9 monitors armed";
              each fire on its own -> "FIRE <monitor_id>"
    watchdog  look(), observation, verdict: one label each, runs of the same kind merged
    bus       nothing; marker shapes carry it
    """
    out: list[dict] = []
    ev = sorted((e for e in events if r0 <= e.t <= r1), key=lambda e: e.t)
    starts = [e for e in ev if e.lane == "executor" and e.kind == "step_start"]
    claims = [e for e in ev if e.lane == "executor" and e.kind == "step_claim"]
    for i, st in enumerate(starts):
        step = st.label.replace("begin ", "", 1)
        nxt = starts[i + 1].t if i + 1 < len(starts) else r1
        end = next((c.t for c in claims if c.t >= st.t and c.label.startswith(step + ":")), nxt)
        out.append({"kind": "band", "lane": "executor", "t0": st.t, "t1": min(end, nxt), "text": step})

    def runs(kind_set, lane):
        cur: list = []
        for e in ev:
            if e.lane != lane or e.kind not in kind_set:
                continue
            if cur and (e.kind != cur[-1].kind or e.t - cur[-1].t > gap_s):
                yield cur
                cur = []
            cur.append(e)
        if cur:
            yield cur

    for grp in runs({"arm", "disarm"}, "monitors"):
        n, kind = len(grp), grp[0].kind
        verb = "armed" if kind == "arm" else "disarmed"
        text = f"{n} monitors {verb}" if n > 1 else f"{verb} {grp[0].label.split()[1] if len(grp[0].label.split()) > 1 else ''}"
        out.append({"kind": kind, "lane": "monitors", "t0": grp[0].t, "t1": grp[-1].t, "text": _short(text),
                    "priority": 3 if kind == "arm" else 6, "color": S.event_color("monitors", kind, "")})
    for e in ev:
        if e.lane == "monitors" and e.kind == "fire":
            parts = e.label.split()
            out.append({"kind": "fire", "lane": "monitors", "t0": e.t, "t1": e.t,
                        "text": _short(" ".join(parts[:2])), "priority": 0,
                        "color": S.event_color("monitors", "fire", e.label)})
    names = {"look": "look()", "observation": "observation", "run_verdict": "verdict", "verdict": "verdict"}
    for grp in runs(set(names), "watchdog"):
        e = grp[0]
        text = names[e.kind] + (f" x{len(grp)}" if len(grp) > 1 else "")
        if e.kind in ("run_verdict", "verdict"):
            text = _short(f"verdict: {e.label}", 26)
        out.append({"kind": e.kind, "lane": "watchdog", "t0": e.t, "t1": grp[-1].t, "text": text,
                    "priority": {"run_verdict": 1, "verdict": 1, "observation": 2}.get(e.kind, 4),
                    "color": S.event_color("watchdog", e.kind, e.label)})
    return out     # a key frame further than this from its requested time is flagged


def frame_times(prep: Prepared) -> list[float]:
    """Key frame times (session seconds) from the story, inside the window."""
    r0, r1 = prep.timemap.r_start, prep.timemap.r_end
    return [float(x) for x in (prep.story.figure_frames_at or []) if r0 <= float(x) <= r1]


def render_figure(prep: Prepared, out: Path, width: str = "double", formats: tuple[str, ...] = ("svg",)) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    S.apply_matplotlib_style()
    W = S.FIG_WIDTH_IN[width]
    single = width == "single"
    has_sig = bool(prep.signals)
    ratios = [1.7] + ([0.8] if has_sig else [])
    legend_h = 0.36 if single else 0.2
    H = sum(ratios) + 0.55 + legend_h
    n_sig = len(prep.signals)
    fig = plt.figure(figsize=(W, H))
    gs = fig.add_gridspec(len(ratios), 1, height_ratios=ratios, hspace=0.12,
                          left=0.2 if single else 0.155,
                          right=(0.86 if single else 0.93) if n_sig > 1 else 0.985,
                          top=1 - 0.3 / H, bottom=(0.36 + legend_h) / H)
    ax_sw = fig.add_subplot(gs[0])
    ax_sig = fig.add_subplot(gs[1], sharex=ax_sw) if has_sig else None
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
    # ---- markers: shape by kind, so the bus needs no text --------------------
    for e in prep.events:
        mk, ms = MARKERS.get(e.kind, ("o", 3.0 if e.kind in BIG_KINDS else 1.2))
        ax_sw.plot([e.t], [ypos[e.lane]], marker=mk, ms=ms, lw=0, zorder=3,
                   color=S.event_color(e.lane, e.kind, e.label))

    # ---- grouped labels ------------------------------------------------------
    axes_w_in = W * (gs.right - gs.left)
    s_per_char = span * (MIN_PT * 0.5 / 72.0) / axes_w_in      # ~0.5 em per character
    groups = group_labels(prep.events, r0, r1, gap_s=GROUP_GAP_FRAC * span)
    band_i = 0
    for g in groups:
        if g["kind"] == "band":
            band_i += 1
            # one band per executor step, its name centred above the lane if it fits
            y = ypos[g["lane"]]
            gap = 0.004 * span
            ax_sw.axvspan(g["t0"] + gap, g["t1"] - gap, ymin=(y + 0.7 - 0.13) / (len(lanes) + 0.4),
                          ymax=(y + 0.7 + 0.13) / (len(lanes) + 0.4), color=S.LANE_COLOR[g["lane"]],
                          alpha=0.10 if band_i % 2 else 0.24, lw=0, zorder=1)
            if len(g["text"]) * s_per_char <= (g["t1"] - g["t0"]) * 1.15:
                ax_sw.text((g["t0"] + g["t1"]) / 2, y + 0.26, g["text"], fontsize=MIN_PT, ha="center",
                           va="center", color=S.LANE_COLOR[g["lane"]], zorder=4, clip_on=True)
    rows: dict[tuple[str, int], list[tuple[float, float]]] = {}
    pad = 0.8 * s_per_char
    for g in sorted((g for g in groups if g["kind"] != "band"),
                    key=lambda g: (g["priority"], g["t0"])):
        w = len(g["text"]) * s_per_char
        if g["t1"] > g["t0"]:
            # a group: a thin bracket under the markers, text starting at the group
            ax_sw.plot([g["t0"], g["t1"]], [ypos[g["lane"]] - 0.12] * 2, color=g["color"], lw=0.6, zorder=2,
                       solid_capstyle="butt")
        right_aligned = g["t0"] + w > r1
        a0, a1 = (g["t1"] - w, g["t1"]) if right_aligned else (g["t0"], g["t0"] + w)
        if a0 < r0:
            continue
        for row, dy in ((0, 0.24), (1, -0.30)):
            taken = rows.setdefault((g["lane"], row), [])
            if all(a1 + pad < b0 or a0 - pad > b1 for b0, b1 in taken):
                taken.append((a0, a1))
                ax_sw.text(g["t1"] if right_aligned else g["t0"], ypos[g["lane"]] + dy, g["text"], fontsize=MIN_PT,
                           ha="right" if right_aligned else "left", va="center", clip_on=True, color=g["color"],
                           zorder=4)
                break

    # ---- legend for marker shapes, one line under the axes --------------------
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], marker=m, ms=s_, lw=0, color=c, label=l) for m, s_, c, l in (
        ("s", 3.2, S.END_STEP_DARK_GREEN, "step begin / end"), ("^", 3.6, S.BUS_GRAY, "gate open"),
        ("D", 3.0, S.WATCHDOG_GREEN, "monitor armed"), ("x", 3.0, S.WATCHDOG_GREEN, "disarmed"),
        ("o", 3.6, S.FAIL_RED, "monitor fire"),
        ("*", 4.5, S.WATCHDOG_GREEN, "look() / observation"))]
    fig.legend(handles=handles, loc="lower center", ncol=6 if not single else 3, fontsize=MIN_PT, frameon=False,
               handletextpad=0.3, columnspacing=1.0, bbox_to_anchor=(0.5, 0.0))
    if ax_sig is None:
        ax_sw.set_xlabel("session time [s]", fontsize=MIN_PT)
    else:
        ax_sw.tick_params(labelbottom=False)

    # ---- key frame marks: numbered ticks on the top edge, matching the exported PNGs
    min_sep = 3.0 * (span * (MIN_PT * 0.6 / 72.0) / (W * (gs.right - gs.left)))   # ~two digits wide
    prev_t, level = None, 0
    for i, t in enumerate(frame_times(prep), 1):
        level = (level + 1) % 2 if prev_t is not None and t - prev_t < min_sep else 0
        prev_t = t
        ax_sw.axvline(t, color=S.MUTED, lw=0.4, ls=":", zorder=1)
        ax_sw.annotate(str(i), xy=(t, len(lanes) - 0.3), xytext=(0, 1.5 + 7.5 * level), textcoords="offset points",
                       fontsize=MIN_PT, ha="center", va="bottom", color=S.INK, annotation_clip=False)
        if ax_sig is not None:
            ax_sig.axvline(t, color=S.MUTED, lw=0.4, ls=":", zorder=1)

    # ---- signals: one y axis per unit ---------------------------------------
    if ax_sig is not None:
        colors = [S.EXECUTOR_BLUE, S.WATCHDOG_GREEN, S.BUS_GRAY]
        for i, sig in enumerate(list(prep.signals.values())[:2]):
            ax = ax_sig if i == 0 else ax_sig.twinx()
            if i:
                ax.spines["right"].set_visible(True)
            ts = [t for t in sig.t if r0 <= t <= r1]
            vs = [v for t, v in zip(sig.t, sig.v) if r0 <= t <= r1]
            ax.plot(ts, vs, color=colors[i], lw=0.7)
            ax.set_ylabel(f"{sig.name} [{sig.unit}]", fontsize=MIN_PT, color=colors[i])
            ax.tick_params(axis="y", colors=colors[i], labelsize=MIN_PT)
            for th in prep.thresholds:
                if th["signal"] == sig.name:
                    ax.axhline(th["value"], color=S.FAIL_RED, lw=0.6, ls="--")
                    ax.text(r0 + 0.005 * span, th["value"], th["label"], fontsize=MIN_PT, color=S.FAIL_RED,
                            ha="left", va="bottom")
        ax_sig.set_xlabel("session time [s]", fontsize=MIN_PT)
        ax_sig.set_xlim(r0, r1)

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    paths = [out.with_suffix(f".{fmt}") for fmt in formats]
    for pth in paths:
        fig.savefig(pth, dpi=200 if pth.suffix == ".png" else None)
    plt.close(fig)
    print(f"wrote {', '.join(str(p) for p in paths)} ({W} in wide)")
    return paths


def export_frames(prep: Prepared, out_dir: Path, cameras: list[str]) -> Path:
    """Copy the frame nearest each key frame time from each camera folder.

    Files are named <n>_t<session seconds>s.<ext> under out_dir/<camera>/, and
    frames.csv records, per file: the key frame number shown on the figure, the
    requested session time, the frame's own session time and wall_time_iso, the
    gap between them, and the source file. Frames are copied byte for byte.
    """
    from .load_session import epoch_to_iso, load_recorder_frames

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for cam in cameras:
        idx = load_recorder_frames(prep.session.dir, cam)
        if not idx.epochs:
            print(f"warning: no frames in frames/{cam}/; skipped")
            continue
        cdir = out_dir / cam
        if cdir.exists():
            shutil.rmtree(cdir)
        cdir.mkdir(parents=True)
        for n, t in enumerate(frame_times(prep), 1):
            want = prep.session.epoch(t)
            j = min(range(len(idx.epochs)), key=lambda i: abs(idx.epochs[i] - want))
            src = Path(idx.paths[j])
            got_t = prep.session.t(idx.epochs[j])
            gap = got_t - t
            dst = cdir / f"{n:02d}_t{t:07.2f}s{src.suffix}"
            shutil.copy2(src, dst)
            if abs(gap) > MAX_FRAME_GAP_S:
                print(f"warning: {cam} frame {n}: nearest frame is {gap:+.2f} s from t = {t:.2f} s")
            rows.append({"key": n, "camera": cam, "file": f"{cam}/{dst.name}", "t_requested_s": f"{t:.3f}",
                         "t_frame_s": f"{got_t:.3f}", "gap_s": f"{gap:+.3f}",
                         "wall_time_iso": epoch_to_iso(idx.epochs[j]),
                         "source": str(src.relative_to(prep.session.dir))})
    with open(out_dir / "frames.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["key", "camera", "file", "t_requested_s", "t_frame_s", "gap_s",
                                           "wall_time_iso", "source"])
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} frames to {out_dir}")
    return out_dir / "frames.csv"
