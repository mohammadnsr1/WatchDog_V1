"""Variable speed playback: a piecewise linear map between session time and video time.

Session ("real") time runs at `idle` x between events and at `event` x inside
[k - pad, k + pad] around every key event k. At each key event the video
freezes for `hold_s` video seconds. Both directions are exported, and every
video instant knows the real session time it shows, so the on-screen badge can
state the true speed and elapsed time.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Segment:
    r0: float          # real (session) seconds
    r1: float
    v0: float          # video seconds
    v1: float
    speed: float       # real seconds per video second; 0.0 for a hold

    @property
    def is_hold(self) -> bool:
        return self.speed == 0.0


@dataclass(frozen=True)
class Moment:
    real: float
    speed: float       # 0.0 while holding
    is_hold: bool

    def badge(self) -> str:
        if self.is_hold:
            return "paused"
        s = self.speed
        return f"{s:g}×" if s >= 1 else f"{s:.2g}×"


class TimeMap:
    def __init__(self, r_start: float, r_end: float, key_times: Iterable[float],
                 idle: float = 8.0, event: float = 1.0, event_pad_s: float = 1.5, hold_s: float = 2.0) -> None:
        if r_end <= r_start:
            raise ValueError(f"empty window [{r_start}, {r_end}]")
        if idle <= 0 or event <= 0:
            raise ValueError("speeds must be positive")
        self.r_start, self.r_end = float(r_start), float(r_end)
        keys = sorted({round(float(k), 3) for k in key_times if r_start <= k <= r_end})
        self.key_times = keys
        # merged slow intervals
        slow: list[list[float]] = []
        for k in keys:
            a, b = max(r_start, k - event_pad_s), min(r_end, k + event_pad_s)
            if slow and a <= slow[-1][1]:
                slow[-1][1] = max(slow[-1][1], b)
            else:
                slow.append([a, b])
        self._slow = slow
        cuts = sorted({self.r_start, self.r_end, *[x for iv in slow for x in iv], *keys})
        segs: list[Segment] = []
        v = 0.0
        held = set()
        for i, a in enumerate(cuts):
            if a in keys and a not in held and hold_s > 0:
                segs.append(Segment(a, a, v, v + hold_s, 0.0))
                v += hold_s
                held.add(a)
            if i + 1 == len(cuts):
                break
            b = cuts[i + 1]
            mid = (a + b) / 2
            speed = event if any(lo <= mid <= hi for lo, hi in slow) else idle
            dv = (b - a) / speed
            segs.append(Segment(a, b, v, v + dv, speed))
            v += dv
        self.segments = segs
        self.duration = v
        self._v_starts = [s.v0 for s in segs]
        self._r_starts = [s.r0 for s in segs]

    # real -> video ------------------------------------------------------
    def real_to_video(self, r: float) -> float:
        """Video time at which real time r first appears (the START of a hold at a key event)."""
        r = min(max(r, self.r_start), self.r_end)
        i = bisect.bisect_right(self._r_starts, r) - 1
        # step back to the first segment that starts at this r (a hold precedes its run segment)
        while i > 0 and self.segments[i - 1].r0 == r:
            i -= 1
        s = self.segments[max(i, 0)]
        if s.is_hold or s.r1 == s.r0:
            return s.v0
        return s.v0 + (r - s.r0) / s.speed

    # video -> real ------------------------------------------------------
    def at_video(self, v: float) -> Moment:
        v = min(max(v, 0.0), self.duration)
        i = max(bisect.bisect_right(self._v_starts, v) - 1, 0)
        s = self.segments[i]
        if s.is_hold:
            return Moment(s.r0, 0.0, True)
        return Moment(min(s.r0 + (v - s.v0) * s.speed, s.r1), s.speed, False)

    def video_to_real(self, v: float) -> float:
        return self.at_video(v).real

    def describe(self) -> str:
        n_hold = sum(1 for s in self.segments if s.is_hold)
        return (f"real {self.r_end - self.r_start:.1f} s -> video {self.duration:.1f} s, "
                f"{len(self.key_times)} key events, {n_hold} holds")


# ---------------------------------------------------------------------------
# Everything a renderer needs, built once per story
# ---------------------------------------------------------------------------

import sys  # noqa: E402
from typing import Any, Optional  # noqa: E402

from . import load_session as ls  # noqa: E402


@dataclass
class Arrow:
    src: "ls.Event"
    dst: "ls.Event"
    color: str


@dataclass
class Prepared:
    story: "ls.Story"
    session: "ls.Session"
    events: list                    # visible events, session time
    key_events: list
    timemap: TimeMap
    frames: "ls.FrameIndex"         # recorder frames, or evidence frames as fallback
    frames_are_evidence: bool
    signals: dict
    thresholds: list
    excerpts: list                  # [{lane, heading, text, t}]
    captions: list                  # [{t, text}]
    arrows: list
    warnings: list


def _pair_wakes(events: list, src_kinds: set[str], reason: str, max_gap: float = 15.0) -> list[tuple[Any, Any]]:
    """Each source event -> the first later WatchDog wake with the given reason."""
    wakes = [e for e in events if e.kind == "wake" and e.label == f"wake: {reason}"]
    out = []
    j = 0
    for e in (x for x in events if x.kind in src_kinds):
        while j < len(wakes) and wakes[j].t < e.t:
            j += 1
        if j < len(wakes) and wakes[j].t - e.t <= max_gap:
            out.append((e, wakes[j]))
    return out


def prepare(story: "ls.Story", need_signals: bool = True) -> Prepared:
    from . import style
    warnings: list[str] = []

    def warn(msg: str) -> None:
        warnings.append(msg)
        print(f"warning: {msg}", file=sys.stderr)

    session = ls.load_session(story.session_dir)
    events = ls.visible_events(session, story)
    if not events:
        raise ValueError("no events inside the story window")
    w = story.window
    r_start = float(w[0]) if w else max(0.0, events[0].t - 1.0)
    r_end = float(w[1]) if w else max(e.end_t or e.t for e in events) + 1.0
    kinds = set(story.key_events or [])
    key_events = [e for e in events if e.kind in kinds]
    sp = story.speed
    tm = TimeMap(r_start, r_end, [e.t for e in key_events], idle=float(sp["idle"]), event=float(sp["event"]),
                 event_pad_s=float(sp["event_pad_s"]), hold_s=float(sp["hold_s"]))

    frames = ls.load_recorder_frames(session.dir, str(story.camera or "main"))
    evidence = False
    if len(frames) == 0:
        frames, evidence = session.evidence_frames, True
        warn(f"no recorder frames for camera {story.camera!r}; using {len(frames)} look() and capture_frames() "
             f"images, which are sparse evidence stills, not continuous video")

    signals = ls.load_signals(session, story.signals or [], warn=warn) if need_signals else {}
    thresholds = ls.resolve_thresholds(session, story, warn=warn)

    excerpts = []
    for ex in story.prompt_excerpts or []:
        t = ls.anchor_time(session, ex)
        if t is None:
            warn(f"prompt excerpt {ex.get('heading')!r}: anchor not found, skipped")
            continue
        excerpts.append({"lane": ex.get("lane", "watchdog"), "heading": ex.get("heading", ""),
                         "text": ex.get("text", ""), "t": t})
    captions = sorted(({"t": float(c["at_t"]), "text": c.get("text", "")} for c in (story.captions or [])
                       if c.get("at_t") is not None), key=lambda c: c["t"])

    arrows: list[Arrow] = []
    arrows += [Arrow(a, b, style.EXECUTOR_BLUE) for a, b in _pair_wakes(events, {"step_start"}, "arming")]
    arrows += [Arrow(a, b, style.FAIL_RED) for a, b in _pair_wakes(events, {"fire"}, "fire")]
    arrows += [Arrow(a, b, style.END_STEP_DARK_GREEN) for a, b in _pair_wakes(events, {"end_step"}, "end_step")]
    halts = [e for e in events if e.kind == "halt"]
    if halts:
        for v in (e for e in events if e.kind in ("verdict", "run_verdict") and e.label.startswith("FAIL")):
            h = next((x for x in halts if x.t >= v.t), None)
            if h is not None:
                arrows.append(Arrow(v, h, style.FAIL_RED))
    return Prepared(story, session, events, key_events, tm, frames, evidence, signals, thresholds,
                    excerpts, captions, arrows, warnings)
