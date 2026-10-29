"""MP4 replay: Isaac frame, agent panels, swimlane timeline, signal strip.

Rendered with Pillow onto a precomputed static layer, encoded H.264
(preset veryfast, CRF 20, yuv420p) through imageio-ffmpeg.
"""
from __future__ import annotations

import bisect
import math
from functools import lru_cache
from pathlib import Path
from typing import Optional

from PIL import Image, ImageDraw

from . import style as S
from .load_session import Event
from .timeline import Moment, Prepared

EXEC_PANEL_KINDS = {"step_start", "step_claim", "tool_call", "look", "run_end", "verdict", "run_verdict", "observation"}
WD_PANEL_KINDS = {"wake", "look", "verdict", "run_verdict", "observation", "halt", "frames_captured", "tool_call"}
LABELED_KINDS = {"step_start", "end_step", "arm", "fire", "verdict", "run_verdict", "look", "halt", "timeout",
                 "observation"}
BIG_KINDS = LABELED_KINDS | {"wake", "look_frame", "step_claim"}
FLASH_KINDS = {"fire"}


def _rgb(h: str) -> tuple[int, int, int]:
    return S.hex_to_rgb(h)


def _text_w(draw: ImageDraw.ImageDraw, text: str, font) -> int:
    return int(draw.textlength(text, font=font))


def _truncate(draw, text: str, font, max_w: int) -> str:
    if _text_w(draw, text, font) <= max_w:
        return text
    while text and _text_w(draw, text + "...", font) > max_w:
        text = text[:-1]
    return text.rstrip() + "..."


def _wrap(draw, text: str, font, max_w: int, max_lines: int) -> list[str]:
    """Greedy word wrap to at most max_lines; the last line ends in '...' if text was cut."""
    words = str(text).split()
    lines: list[str] = []
    cur = ""
    used = 0
    for w in words:
        trial = f"{cur} {w}".strip()
        if _text_w(draw, trial, font) <= max_w or not cur:
            cur = trial
            used += 1
            continue
        lines.append(cur)
        if len(lines) == max_lines:
            cur = ""
            break
        cur = w
        used += 1
    if cur and len(lines) < max_lines:
        lines.append(cur)
    lines = [_truncate(draw, ln, font, max_w) for ln in lines]
    if used < len(words) and lines:
        lines[-1] = _truncate(draw, lines[-1] + " ...", font, max_w)
    return lines


@lru_cache(maxsize=48)
def _fitted(path: str, w: int, h: int) -> Image.Image:
    img = Image.open(path).convert("RGB")
    scale = min(w / img.width, h / img.height)
    return img.resize((max(1, int(img.width * scale)), max(1, int(img.height * scale))), Image.BILINEAR)


def _dashed_hline(draw, x0: int, x1: int, y: int, color, dash: int = 10, gap: int = 6, width: int = 2) -> None:
    x = x0
    while x < x1:
        draw.line([(x, y), (min(x + dash, x1), y)], fill=color, width=width)
        x += dash + gap


def _arrow(draw, p0, p1, color, width: int = 2, head: int = 9) -> None:
    draw.line([p0, p1], fill=color, width=width)
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    left = (p1[0] - head * math.cos(ang - 0.45), p1[1] - head * math.sin(ang - 0.45))
    right = (p1[0] - head * math.cos(ang + 0.45), p1[1] - head * math.sin(ang + 0.45))
    draw.polygon([p1, left, right], fill=color)


class Compositor:
    def __init__(self, prep: Prepared) -> None:
        self.p = prep
        tm = prep.timemap
        self.events = prep.events
        self.ev_t = [e.t for e in self.events]
        self.vt = {id(e): tm.real_to_video(e.t) for e in self.events}
        self.flash_events = sorted(((self.vt[id(e)], e) for e in self.events
                                    if e.kind in FLASH_KINDS or (e.kind in ("verdict", "run_verdict") and e.label.startswith("FAIL"))),
                                   key=lambda x: x[0])
        self.look_frames = sorted(((self.vt[id(e)], e) for e in self.events if e.kind == "look_frame" and e.image),
                                  key=lambda x: x[0])
        self.excerpts = [dict(x, v=tm.real_to_video(x["t"])) for x in prep.excerpts]
        self.captions = [dict(c, v=tm.real_to_video(c["t"])) for c in prep.captions]
        self.panel_events = {
            "executor": [e for e in self.events if e.lane == "executor" and e.kind in EXEC_PANEL_KINDS],
            "watchdog": [e for e in self.events if e.lane == "watchdog" and e.kind in WD_PANEL_KINDS],
        }
        self.panel_t = {k: [e.t for e in v] for k, v in self.panel_events.items()}
        self.signal_rows = list(prep.signals.values())[:3]
        self.sig_range = {}
        for sig in self.signal_rows:
            vals = list(sig.v) + [th["value"] for th in prep.thresholds if th["signal"] == sig.name]
            lo, hi = min(vals), max(vals)
            pad = (hi - lo) * 0.08 or 1.0
            self.sig_range[sig.name] = (lo - pad, hi + pad)
        self.static = self._static_layer()

    # ---- geometry ---------------------------------------------------------
    @property
    def lane_h(self) -> float:
        return (S.SWIM_BOX[3] - S.SWIM_BOX[1]) / len(S.LANES)

    def lane_y(self, lane: str) -> int:
        i = S.LANES.index(lane) if lane in S.LANES else 1
        return int(S.SWIM_BOX[1] + (i + 0.5) * self.lane_h)

    def x_of(self, t: float, real: float) -> float:
        x0, x1 = S.SWIM_BOX[0] + S.LANE_LABEL_W, S.SWIM_BOX[2] - 8
        lo = real - S.SWIM_WINDOW_S / 2
        return x0 + (t - lo) / S.SWIM_WINDOW_S * (x1 - x0)

    # ---- static layer ----------------------------------------------------
    def _static_layer(self) -> Image.Image:
        img = Image.new("RGB", (S.VIDEO_W, S.VIDEO_H), _rgb(S.CANVAS_BG))
        d = ImageDraw.Draw(img)
        f_title = S.pil_font(26, bold=True)
        f_lane = S.pil_font(19, bold=True)
        f_small = S.pil_font(17)
        d.rectangle(S.FRAME_BOX, fill=(32, 34, 38))
        for box, title, color in ((S.EXEC_PANEL, "Executor", S.EXECUTOR_BLUE), (S.WD_PANEL, "WatchDog", S.WATCHDOG_GREEN)):
            d.rounded_rectangle(box, radius=10, fill=_rgb(S.PANEL_BG), outline=_rgb(color), width=3)
            d.text((box[0] + 16, box[1] + 10), title, font=f_title, fill=_rgb(color))
        d.rectangle(S.SWIM_BOX, fill=_rgb(S.CANVAS_BG), outline=_rgb(S.GRID))
        for i, lane in enumerate(S.LANES):
            y0 = int(S.SWIM_BOX[1] + i * self.lane_h)
            y1 = int(S.SWIM_BOX[1] + (i + 1) * self.lane_h)
            if i % 2 == 0:
                d.rectangle((S.SWIM_BOX[0] + 1, y0, S.SWIM_BOX[2] - 1, y1), fill=(250, 251, 252))
            d.line([(S.SWIM_BOX[0], y1), (S.SWIM_BOX[2], y1)], fill=_rgb(S.GRID))
            title = S.LANE_TITLES[lane]
            col = _rgb(S.LANE_COLOR[lane])
            d.rectangle((S.SWIM_BOX[0], y0, S.SWIM_BOX[0] + 6, y1), fill=col)
            lines = _wrap(d, title, f_lane, S.LANE_LABEL_W - 26, 2)
            ty = (y0 + y1) / 2 - len(lines) * 12
            for ln in lines:
                d.text((S.SWIM_BOX[0] + 16, ty), ln, font=f_lane, fill=col)
                ty += 24
        d.line([(S.SWIM_BOX[0] + S.LANE_LABEL_W, S.SWIM_BOX[1]), (S.SWIM_BOX[0] + S.LANE_LABEL_W, S.SWIM_BOX[3])],
               fill=_rgb(S.GRID), width=2)
        d.rectangle(S.SIGNAL_BOX, outline=_rgb(S.GRID))
        if not self.signal_rows:
            d.text((S.SIGNAL_BOX[0] + 16, S.SIGNAL_BOX[1] + 10), "No signals selected in the story.", font=f_small,
                   fill=_rgb(S.MUTED))
        else:
            rh = (S.SIGNAL_BOX[3] - S.SIGNAL_BOX[1]) / len(self.signal_rows)
            for i, sig in enumerate(self.signal_rows):
                y = S.SIGNAL_BOX[1] + i * rh
                d.text((S.SIGNAL_BOX[0] + 16, y + 6), sig.name, font=f_small, fill=_rgb(S.INK))
                d.text((S.SIGNAL_BOX[0] + 16, y + 28), f"[{sig.unit}]", font=f_small, fill=_rgb(S.MUTED))
        return img

    # ---- dynamic layers ---------------------------------------------------
    def _frame_panel(self, img: Image.Image, d: ImageDraw.ImageDraw, m: Moment, v: float) -> None:
        box = S.FRAME_BOX
        bw, bh = box[2] - box[0], box[3] - box[1]
        path = self.p.frames.at(self.p.session.epoch(m.real))
        f_mid = S.pil_font(22)
        if path:
            fr = _fitted(path, bw, bh)
            img.paste(fr, (box[0] + (bw - fr.width) // 2, box[1] + (bh - fr.height) // 2))
            if self.p.frames_are_evidence:
                tag = "evidence still (look or capture), not continuous video"
                d.rounded_rectangle((box[0] + 12, box[3] - 44, box[0] + 24 + _text_w(d, tag, S.pil_font(17)), box[3] - 12),
                                    radius=6, fill=(0, 0, 0))
                d.text((box[0] + 18, box[3] - 40), tag, font=S.pil_font(17), fill=(230, 230, 230))
        else:
            msg = "No camera frame recorded yet at this time"
            d.text((box[0] + (bw - _text_w(d, msg, f_mid)) // 2, box[1] + bh // 2 - 12), msg, font=f_mid, fill=(200, 200, 200))
        # title
        story = self.p.story
        if story.title:
            f_t = S.pil_font(24, bold=True)
            f_s = S.pil_font(18)
            w = max(_text_w(d, story.title, f_t), _text_w(d, story.subtitle or "", f_s)) + 28
            d.rounded_rectangle((box[0] + 12, box[1] + 12, box[0] + 12 + w, box[1] + (70 if story.subtitle else 46)),
                                radius=8, fill=(0, 0, 0))
            d.text((box[0] + 26, box[1] + 18), story.title, font=f_t, fill=(255, 255, 255))
            if story.subtitle:
                d.text((box[0] + 26, box[1] + 46), story.subtitle, font=f_s, fill=(210, 210, 210))
        # speed badge: always visible, honest about compression
        f_b = S.pil_font(30, bold=True)
        f_r = S.pil_font(20)
        badge = m.badge()
        real = f"real t = {m.real:7.1f} s"
        bwid = max(_text_w(d, badge, f_b), _text_w(d, real, f_r)) + 30
        x1 = box[2] - 12
        d.rounded_rectangle((x1 - bwid, box[1] + 12, x1, box[1] + 92), radius=10,
                            fill=_rgb(S.FAIL_RED) if m.is_hold else (0, 0, 0))
        d.text((x1 - bwid + 15, box[1] + 18), badge, font=f_b, fill=(255, 255, 255))
        d.text((x1 - bwid + 15, box[1] + 60), real, font=f_r, fill=(235, 235, 235))
        # red flash + verdict or fire card
        recent = [(vt, e) for vt, e in self.flash_events if vt <= v]
        if recent:
            vt, e = recent[-1]
            if v - vt <= S.FLASH_VIDEO_S:
                d.rectangle(box, outline=_rgb(S.FAIL_RED), width=10)
            if v - vt <= S.CARD_VIDEO_S:
                self._card(d, e)

    def _card(self, d, e: Event) -> None:
        box = S.FRAME_BOX
        f_h = S.pil_font(22, bold=True)
        f_b = S.pil_font(19)
        w = box[2] - box[0] - 80
        lines = _wrap(d, e.detail, f_b, w - 32, 2)
        h = 46 + 26 * len(lines)
        x0, y1 = box[0] + 40, box[3] - 120
        d.rounded_rectangle((x0, y1 - h, x0 + w, y1), radius=10, fill=(255, 255, 255), outline=_rgb(S.FAIL_RED), width=4)
        d.text((x0 + 16, y1 - h + 10), f"{e.label}   at {e.t:.1f} s", font=f_h, fill=_rgb(S.FAIL_RED))
        for i, ln in enumerate(lines):
            d.text((x0 + 16, y1 - h + 42 + 26 * i), ln, font=f_b, fill=_rgb(S.INK))

    def _panel(self, d, lane: str, box, color: str, m: Moment, v: float) -> None:
        f_line = S.pil_font(18)
        f_head = S.pil_font(18, bold=True)
        x0, y = box[0] + 16, box[1] + 50
        maxw = box[2] - box[0] - 32
        active = [x for x in self.excerpts if x["lane"] == lane and x["v"] <= v <= x["v"] + S.EXCERPT_VIDEO_S]
        if active:
            ex = active[-1]
            body = _wrap(d, ex["text"], f_line, maxw - 24, 3)
            h = 40 + 24 * len(body)
            d.rounded_rectangle((x0 - 4, y, box[2] - 12, y + h), radius=8, fill=_rgb(S.HIGHLIGHT_BG), outline=_rgb(color), width=2)
            d.text((x0 + 8, y + 8), ex["heading"], font=f_head, fill=_rgb(color))
            for i, ln in enumerate(body):
                d.text((x0 + 8, y + 34 + 24 * i), ln, font=f_line, fill=_rgb(S.INK))
            y += h + 8
        evs = self.panel_events[lane]
        n = bisect.bisect_right(self.panel_t[lane], m.real)
        room = max(1, int((box[3] - 12 - y) // 30))
        shown = evs[max(0, n - min(S.PANEL_LINES, room)):n]
        y = box[3] - 12 - 30 * len(shown)
        for e in shown:
            col = S.event_color(e.lane, e.kind, e.label) if e.kind in ("verdict", "run_verdict", "halt") else S.INK
            d.text((x0, y), f"{e.t:7.1f} s", font=f_line, fill=_rgb(S.MUTED))
            d.text((x0 + 92, y), _truncate(d, e.label, f_line, maxw - 92), font=f_line, fill=_rgb(col))
            y += 30

    def _swimlanes(self, img, d, m: Moment, v: float) -> None:
        real = m.real
        lo, hi = real - S.SWIM_WINDOW_S / 2, real + S.SWIM_WINDOW_S / 2
        x_left, x_right = S.SWIM_BOX[0] + S.LANE_LABEL_W, S.SWIM_BOX[2] - 8
        f_tick = S.pil_font(15)
        f_lab = S.pil_font(15)
        for tick in range(int(math.ceil(lo / 5.0)) * 5, int(hi) + 1, 5):
            x = self.x_of(tick, real)
            if x_left <= x <= x_right:
                d.line([(x, S.SWIM_BOX[1]), (x, S.SWIM_BOX[3])], fill=(236, 239, 242))
                d.text((x + 3, S.SWIM_BOX[3] - 20), f"{tick} s", font=f_tick, fill=_rgb(S.MUTED))
        for a in self.p.arrows:
            if (lo <= a.src.t <= hi) or (lo <= a.dst.t <= hi):
                p0 = (min(max(self.x_of(a.src.t, real), x_left), x_right), self.lane_y(a.src.lane))
                p1 = (min(max(self.x_of(a.dst.t, real), x_left), x_right), self.lane_y(a.dst.lane))
                _arrow(d, p0, p1, _rgb(a.color))
        i0 = bisect.bisect_left(self.ev_t, lo)
        i1 = bisect.bisect_right(self.ev_t, hi)
        last_label_end: dict[str, float] = {}
        for e in self.events[i0:i1]:
            x = self.x_of(e.t, real)
            if not (x_left <= x <= x_right):
                continue
            y = self.lane_y(e.lane)
            col = _rgb(S.event_color(e.lane, e.kind, e.label))
            r = 7 if e.kind in BIG_KINDS else 3
            if e.kind == "tool_call" and e.end_t is not None:
                xe = min(self.x_of(e.end_t, real), x_right)
                if xe - x > 2:
                    d.rectangle((x, y + 10, xe, y + 15), fill=col)
            d.ellipse((x - r, y - r, x + r, y + r), fill=col)
            if e.kind in LABELED_KINDS or (e.kind == "wake" and e.label != "wake: ?"):
                txt = _truncate(d, e.label, f_lab, 260)
                if x > last_label_end.get(e.lane, -1e9) + 6:
                    d.text((x + 9, y - 26), txt, font=f_lab, fill=col if e.kind != "tool_call" else _rgb(S.INK))
                    last_label_end[e.lane] = x + 9 + _text_w(d, txt, f_lab)
        xp = self.x_of(real, real)
        d.line([(xp, S.SWIM_BOX[1]), (xp, S.SWIM_BOX[3])], fill=_rgb(S.INK), width=3)
        # look() popups
        for vt, e in self.look_frames:
            if 0 <= v - vt <= S.LOOK_POPUP_VIDEO_S:
                tw, th = 256, 144
                thumb = _fitted(e.image, tw, th)
                y_lane = self.lane_y(e.lane)
                x = int(min(max(xp - tw // 2, x_left), x_right - tw))
                y = int(y_lane - th - 34)
                d.rectangle((x - 3, y - 3, x + thumb.width + 3, y + thumb.height + 27), fill=_rgb(S.LANE_COLOR[e.lane]))
                img.paste(thumb, (x, y))
                d.text((x + 4, y + thumb.height + 3), f"look() at {e.t:.1f} s", font=S.pil_font(16), fill=(255, 255, 255))

    def _signals(self, d, m: Moment) -> None:
        if not self.signal_rows:
            return
        real = m.real
        lo, hi = real - S.SWIM_WINDOW_S / 2, real + S.SWIM_WINDOW_S / 2
        x_left, x_right = S.SIGNAL_BOX[0] + S.LANE_LABEL_W, S.SIGNAL_BOX[2] - 8
        rh = (S.SIGNAL_BOX[3] - S.SIGNAL_BOX[1]) / len(self.signal_rows)
        f = S.pil_font(15)
        for i, sig in enumerate(self.signal_rows):
            y0 = S.SIGNAL_BOX[1] + i * rh + 6
            y1 = S.SIGNAL_BOX[1] + (i + 1) * rh - 6
            vlo, vhi = self.sig_range[sig.name]
            ymap = lambda val: y1 - (val - vlo) / (vhi - vlo) * (y1 - y0)  # noqa: E731
            for th in self.p.thresholds:
                if th["signal"] == sig.name:
                    yy = ymap(th["value"])
                    _dashed_hline(d, x_left, x_right, int(yy), _rgb(S.FAIL_RED))
                    lab = f"{th['label']} ({th['value']:g})"
                    d.text((x_right - _text_w(d, lab, f) - 4, yy - 19), lab, font=f, fill=_rgb(S.FAIL_RED))
            j0 = bisect.bisect_left(sig.t, lo)
            j1 = bisect.bisect_right(sig.t, hi)
            step = max(1, (j1 - j0) // 1500)
            pts = [(self.x_of(sig.t[j], real), ymap(sig.v[j])) for j in range(j0, j1, step)]
            if len(pts) >= 2:
                d.line(pts, fill=_rgb(S.EXECUTOR_BLUE), width=2)
            xp = self.x_of(real, real)
            d.line([(xp, y0), (xp, y1)], fill=_rgb(S.INK), width=2)

    def _caption(self, img, v: float) -> None:
        cur = [c for c in self.captions if c["v"] <= v <= c["v"] + S.CAPTION_VIDEO_S]
        if not cur:
            return
        box = S.FRAME_BOX
        overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
        od = ImageDraw.Draw(overlay)
        f = S.pil_font(26)
        lines = _wrap(od, cur[-1]["text"], f, box[2] - box[0] - 80, 2)
        h = 24 + 34 * len(lines)
        od.rectangle((box[0], box[3] - h - 10, box[2], box[3] - 10), fill=(0, 0, 0, 170))
        for k, ln in enumerate(lines):
            od.text((box[0] + 40, box[3] - h + 2 + 34 * k), ln, font=f, fill=(255, 255, 255, 255))
        img.paste(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))

    def frame_at(self, v: float) -> Image.Image:
        m = self.p.timemap.at_video(v)
        img = self.static.copy()
        d = ImageDraw.Draw(img)
        self._frame_panel(img, d, m, v)
        self._panel(d, "executor", S.EXEC_PANEL, S.EXECUTOR_BLUE, m, v)
        self._panel(d, "watchdog", S.WD_PANEL, S.WATCHDOG_GREEN, m, v)
        self._swimlanes(img, d, m, v)
        self._signals(d, m)
        self._caption(img, v)
        return img


# ---------------------------------------------------------------------------
# entry points
# ---------------------------------------------------------------------------

def render_video(prep: Prepared, out: Path, progress: bool = True) -> Path:
    import imageio_ffmpeg

    comp = Compositor(prep)
    n = int(math.ceil(prep.timemap.duration * S.FPS))
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio_ffmpeg.write_frames(
        str(out), (S.VIDEO_W, S.VIDEO_H), fps=S.FPS, codec="libx264", pix_fmt_in="rgb24",
        pix_fmt_out="yuv420p", quality=None, macro_block_size=1,
        output_params=["-preset", "veryfast", "-crf", "20", "-movflags", "+faststart"],
    )
    writer.send(None)
    try:
        for i in range(n):
            writer.send(comp.frame_at(i / S.FPS).tobytes())
            if progress and i % (S.FPS * 10) == 0:
                print(f"  frame {i}/{n} ({100 * i / max(n, 1):.0f}%)", flush=True)
    finally:
        writer.close()
    print(f"wrote {out} ({n} frames, {prep.timemap.describe()})")
    return out


def render_preview(prep: Prepared, n_frames: int, out_dir: Path) -> list[Path]:
    comp = Compositor(prep)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    dur = prep.timemap.duration
    times = sorted({dur * k / max(n_frames - 1, 1) for k in range(n_frames)})
    paths = []
    for i, v in enumerate(times):
        p = out_dir / f"preview_{i:03d}_v{v:06.1f}s.png"
        comp.frame_at(v).save(p)
        paths.append(p)
    print(f"wrote {len(paths)} preview frames to {out_dir} ({prep.timemap.describe()})")
    return paths


def render_poster(prep: Prepared, out: Path, at_video: Optional[float] = None) -> Path:
    comp = Compositor(prep)
    if at_video is None:
        firsts = [comp.vt[id(e)] for e in prep.key_events]
        at_video = (min(firsts) + 0.5) if firsts else prep.timemap.duration / 2
    out = Path(out)
    comp.frame_at(min(at_video, prep.timemap.duration)).save(out, "JPEG", quality=88)
    return out
