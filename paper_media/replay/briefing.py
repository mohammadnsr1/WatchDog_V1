"""Task hand-over card shown before the run in every video.

One full frame: the task both agents receive, then side by side what the executor
is told to do and what the WatchDog is told to check. The part order and the step
count come from the executor task YAML named in agent_config_executor.json; the
wording of the duties is per task (TASK_BRIEFS) and per scenario (SCENARIO_BRIEFS),
paraphrased from agents/tasks/<task>.{executor,watchdog}.yaml and the WatchDog
system prompts. Keep it short enough to read in BRIEFING_VIDEO_S.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml
from PIL import Image, ImageDraw

from . import style as S
from .icons import draw_pil
from .load_session import REPO_ROOT, Session, agents_text

# What the agents are told, per task name (the executor task YAML's `name`).
TASK_BRIEFS = {
    "fmb_assembly_2": {
        "headline": "Assemble the FMB board: seat four parts on the base1 fixture, in order.",
        "executor": [
            "Follows the step plan: for each part, pick it, approach the placement frame, seat it, release and retreat.",
            "Gets grasp widths, approach offsets and the placement frames; queries each frame live.",
            "Must seat each part within its registry tolerance, grip it securely with no slip, never exceed a part's clamp-effort limit, never drive a part into base1 or a seated part, and leave every other part where it is.",
            "Reports success or failure for every step.",
        ],
        "watchdog": [
            "Gets only what must be true at the end of each step, never how the executor gets there.",
            "At each place step: the part sits at its placement frame within tolerance, was gripped at the right width and lifted clear, and a grasp reported as holding the part really held that part; base1 did not move.",
            "Throughout: clamp effort within each part's limit, no slip in the gripper, untouched parts stay put.",
            "Watches the work area for the whole run and reports anything unexpected: an unlisted object, an object that moved by itself, someone or something entering and leaving.",
        ],
    },
}
TASK_BRIEFS["fmb_assembly"] = TASK_BRIEFS["fmb_assembly_2"]

# How the WatchDog does it, per scenario (agent_config_watchdog.json `scenario`).
SCENARIO_BRIEFS = {
    "s1": "Arms monitors from the built-in library; every threshold comes from the contract and part registry.",
    "s2": "Arms built-in monitors and writes extra checks of its own; thresholds from the contract and part registry.",
    "s3": "Writes every monitor check itself; thresholds from the contract and part registry.",
    "s4": "No WatchDog in this run: the executor arms the same monitors and judges its own work.",
}
GATE_LINE = "Before each step the executor announces it and waits at the step gate until the WatchDog's monitors are armed."
GATE_LINE_S4 = "There is no step gate: nobody else is watching."
# The strip under the two columns: how the agents talk during the run, one chip per stage.
CYCLE = [
    ("Executor announces the next step", S.EXECUTOR_BLUE, "megaphone"),
    ("WatchDog arms monitors for it", S.WATCHDOG_GREEN, "shield"),
    ("Step gate released: the robot moves", S.BUS_GRAY, "gate"),
    ("A monitor fires on a limit", S.FAIL_RED, "bolt"),
    ("WatchDog looks, judges, records", S.WATCHDOG_GREEN, "gavel"),
]
CYCLE_S4 = [
    ("Executor arms its own monitors", S.EXECUTOR_BLUE, "shield"),
    ("The robot moves", S.BUS_GRAY, "arm"),
    ("A monitor fires on a limit", S.FAIL_RED, "bolt"),
    ("Executor polls its monitors when it chooses", S.EXECUTOR_BLUE, "eye"),
    ("Executor judges its own work", S.EXECUTOR_BLUE, "gavel"),
]
HIDDEN_LINE = "Hidden from both agents: the operator adds objects to the scene during the run (shown as ground truth)."


def _task_yaml(session: Session) -> Optional[dict]:
    cfg = session.agent_config.get("executor") or {}
    p = cfg.get("executor_task_path")
    if not p:
        return None
    path = Path(p)
    if not path.exists():
        path = REPO_ROOT / "agents" / "tasks" / path.name
    if not path.exists():
        return None
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def briefing_text(session: Session, has_ground_truth: bool) -> dict:
    """Everything the card shows, as text. Falls back to generic lines for an unknown task."""
    task = _task_yaml(session) or {}
    name = str(task.get("name") or "").removesuffix("_s4")
    steps = task.get("steps") or []
    parts: list[str] = []
    for st in steps:
        o = st.get("object_id")
        if o and o not in parts:
            parts.append(o)
    brief = TASK_BRIEFS.get(name, {
        "headline": f"Task {name or '?'}: {len(steps)} steps.",
        "executor": ["Follows the step plan and reports each step."],
        "watchdog": ["Checks each step against the task's requirements."],
    })
    scenario = str((session.agent_config.get("watchdog") or {}).get("scenario") or "")
    s4 = "watchdog" not in session.agent_config
    if s4:
        scenario = "s4"
    return {
        "task_name": name,
        "headline": brief["headline"],
        "parts": parts,
        "n_steps": len(steps),
        "executor": brief["executor"],
        "watchdog": brief["watchdog"],
        "scenario": scenario,
        "scenario_line": SCENARIO_BRIEFS.get(scenario, ""),
        "gate_line": GATE_LINE_S4 if s4 else GATE_LINE,
        "hidden_line": HIDDEN_LINE if has_ground_truth else "",
        "agents": agents_text(session),
        "s4": s4,
    }


def _wrap(d: ImageDraw.ImageDraw, text: str, font, max_w: int) -> list[str]:
    words = text.split()
    lines, cur = [], ""
    for w in words:
        t = f"{cur} {w}".strip()
        if d.textlength(t, font=font) <= max_w or not cur:
            cur = t
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def _bullets(d, x: int, y: int, items: list[str], font, color, max_w: int, gap: int = 10) -> int:
    lh = font.size + 7
    for item in items:
        lines = _wrap(d, item, font, max_w - 28)
        d.ellipse((x + 4, y + lh // 2 - 4, x + 12, y + lh // 2 + 4), fill=color)
        for ln in lines:
            d.text((x + 26, y), ln, font=font, fill=_rgb(S.INK))
            y += lh
        y += gap
    return y


def _rgb(h: str):
    return S.hex_to_rgb(h)


TINT = {S.EXECUTOR_BLUE: "#e8f0fa", S.WATCHDOG_GREEN: "#e6f5ec", S.BUS_GRAY: "#f1f3f5", S.FAIL_RED: "#fdecec"}


def _panel(d, box, color, title, tag, f_title, f_tag, radius=16, bar=58, icon=None):
    """Tinted rounded panel with a filled header bar, white title and italic tag; returns body top y."""
    x0, y0, x1, y1 = box
    d.rounded_rectangle((x0 + 5, y0 + 5, x1 + 5, y1 + 5), radius=radius, fill=(232, 234, 237))   # soft shadow
    d.rounded_rectangle(box, radius=radius, fill=_rgb(TINT.get(color, S.PANEL_BG)), outline=_rgb(color), width=3)
    d.rounded_rectangle((x0, y0, x1, y0 + bar), radius=radius, fill=_rgb(color))
    d.rectangle((x0, y0 + bar // 2, x1, y0 + bar), fill=_rgb(color))
    tx = x0 + 26
    if icon:
        draw_pil(d, icon, x0 + 24, y0 + 12, bar - 24, (255, 255, 255), width=3)
        tx += bar - 10
    d.text((tx, y0 + bar // 2 - f_title.size // 2 - 2), title, font=f_title, fill=(255, 255, 255))
    if tag:
        tw = d.textlength(tag, font=f_tag)
        d.text((x1 - 26 - tw, y0 + bar // 2 - f_tag.size // 2), tag, font=f_tag, fill=(255, 255, 255))
    return y0 + bar + 22


def render_briefing(session: Session, has_ground_truth: bool, title: str = "") -> Image.Image:
    """The static card, VIDEO_W x VIDEO_H. Same look as figures/task_handover.py."""
    b = briefing_text(session, has_ground_truth)
    img = Image.new("RGB", (S.VIDEO_W, S.VIDEO_H), _rgb(S.CANVAS_BG))
    d = ImageDraw.Draw(img)
    f_kicker = S.pil_font(22, bold=True)
    f_head = S.pil_font(40, bold=True)
    f_task = S.pil_font(27)
    f_col = S.pil_font(30, bold=True)
    f_tag = S.pil_font(20)
    f_body = S.pil_font(23)
    f_foot = S.pil_font(21)
    M = 72

    d.text((M, 44), "BEFORE THE RUN: THE TASK IS HANDED TO BOTH AGENTS", font=f_kicker, fill=_rgb(S.MUTED))
    y = 84
    for ln in _wrap(d, b["headline"], f_head, S.VIDEO_W - 2 * M):
        d.text((M, y), ln, font=f_head, fill=_rgb(S.INK))
        y += 48
    y += 6
    order = "  \u2192  ".join(b["parts"]) if b["parts"] else ""
    task_line = f"{b['n_steps']} steps" + (f":  {order}" if order else "")
    for ln in _wrap(d, task_line, f_task, S.VIDEO_W - 2 * M):
        d.text((M, y), ln, font=f_task, fill=_rgb(S.MUTED))
        y += 34
    y += 22

    # two columns
    col_w = (S.VIDEO_W - 2 * M - 48) // 2
    boxes = [
        (M, "Executor Agent", "does the work: the only agent that can move the robot", S.EXECUTOR_BLUE, b["executor"], "arm"),
        (M + col_w + 48, "WatchDog Agent", "checks the work; cannot touch the robot",
         S.WATCHDOG_GREEN, b["watchdog"] + ([b["scenario_line"]] if b["scenario_line"] else []), "eye"),
    ]
    if b["s4"]:
        boxes[1] = (boxes[1][0], "No WatchDog", "scenario S4: the executor monitors itself", S.WATCHDOG_GREEN,
                    [b["scenario_line"]] + b["watchdog"][1:], "eye")
    top, bottom = y, 740
    for x, name, tag, color, items, icon in boxes:
        yy = _panel(d, (x, top, x + col_w, bottom), color, name, tag, f_col, f_tag, icon=icon)
        _bullets(d, x + 28, yy, items, f_body, _rgb(color), col_w - 56)

    # the cycle strip: numbered chips joined by arrows
    y = bottom + 24
    d.text((M, y), "DURING THE RUN, EVERY STEP", font=f_kicker, fill=_rgb(S.MUTED))
    y += 36
    chips = CYCLE_S4 if b["s4"] else CYCLE
    f_chip = S.pil_font(20, bold=True)
    f_num = S.pil_font(18, bold=True)
    gap = 34
    chip_w = (S.VIDEO_W - 2 * M - gap * (len(chips) - 1)) // len(chips)
    chip_h = 74
    for i, (text, color, icon) in enumerate(chips):
        x = M + i * (chip_w + gap)
        d.rounded_rectangle((x, y, x + chip_w, y + chip_h), radius=12, fill=(255, 255, 255), outline=_rgb(color), width=3)
        cx, cy = x + 28, y + chip_h // 2
        d.ellipse((cx - 16, cy - 16, cx + 16, cy + 16), fill=_rgb(color))
        nw = d.textlength(str(i + 1), font=f_num)
        d.text((cx - nw / 2, cy - 11), str(i + 1), font=f_num, fill=(255, 255, 255))
        draw_pil(d, icon, x + chip_w - 48, y + chip_h // 2 - 17, 34, _rgb(color), width=3)
        lines = _wrap(d, text, f_chip, chip_w - 112)
        ty = y + chip_h / 2 - len(lines) * 13
        for ln in lines:
            d.text((x + 56, ty), ln, font=f_chip, fill=_rgb(color))
            ty += 26
        if i < len(chips) - 1:
            ax0, ax1, ay = x + chip_w + 6, x + chip_w + gap - 6, y + chip_h // 2
            d.line([(ax0, ay), (ax1, ay)], fill=_rgb(S.MUTED), width=3)
            d.polygon([(ax1, ay), (ax1 - 9, ay - 6), (ax1 - 9, ay + 6)], fill=_rgb(S.MUTED))
    y += chip_h + 16

    # footer: the step gate, hidden ground truth, models
    d.text((M, y), b["gate_line"], font=f_foot, fill=_rgb(S.INK))
    y += 30
    if b["hidden_line"]:
        d.text((M, y), b["hidden_line"], font=f_foot, fill=_rgb(S.GT_AMBER))
        y += 30
    d.text((M, y), b["agents"], font=f_foot, fill=_rgb(S.MUTED))
    if title:
        tw = d.textlength(title, font=f_foot)
        d.text((S.VIDEO_W - M - tw, S.VIDEO_H - 44), title, font=f_foot, fill=_rgb(S.MUTED))
    return img
