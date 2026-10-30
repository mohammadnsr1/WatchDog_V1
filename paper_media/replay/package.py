"""Package one story into paper_media/outputs/<media_id>/.

    outputs/<media_id>/
      media.yaml          provenance, labels and tags (written last)
      story.yaml          the story exactly as used
      figure/             <media_id>_double.pdf/.svg and <media_id>_single.pdf/.svg
      frames/             <camera>/<n>_t<session s>s.<ext> and frames.csv
      video/              <media_id>.mp4 and <media_id>_poster.jpg (unless --no-video)

The figure's numbered ticks are the frame numbers <n>. Every value in
media.yaml is read from the session or the repository; nothing is estimated.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

import yaml

from .load_session import Story, epoch_to_iso, read_jsonl, wall_ns_to_iso
from .timeline import Prepared, prepare

REPO = Path(__file__).resolve().parents[2]
MEDIA_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git(*args: str) -> str | None:
    try:
        return subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _scored_sessions() -> dict[str, str]:
    """session id -> batch id, for every session listed in a paper_results batch."""
    out = {}
    for by in sorted((REPO / "paper_results" / "batches").glob("*/batch.yaml")):
        text = by.read_text(encoding="utf-8")
        for sid in re.findall(r"\b(\d{8}T\d{6}Z)\b", text):
            out.setdefault(sid, by.parent.name)
    return out


def _orchestrator(session_dir: Path) -> dict:
    for f in sorted((session_dir / "orchestrator").glob("*.jsonl")):
        for _, r in read_jsonl(f, []):
            if r.get("type") == "orchestrator_start":
                return {"guard_mode": r.get("guard_mode"), "gate_timeout_sec": r.get("gate_timeout_sec")}
    return {}


def _injections(session_dir: Path) -> list[dict]:
    out: dict[str, dict] = {}
    for _, r in read_jsonl(session_dir / "ground_truth_injections.jsonl", []):
        if r.get("scenario") == "tracker":
            continue
        name = r.get("name")
        d = out.setdefault(name, {"name": name, "event_kind": r.get("event_kind"), "scenario": r.get("scenario"),
                                  "label": r.get("label"), "expected_impact": r.get("expected_impact"),
                                  "trigger": r.get("trigger"), "actions": []})
        d["actions"].append({"action": r.get("action"), "wall_time_iso": r.get("wall_time_iso")})
    return list(out.values())


def _verdict(session_dir: Path) -> str | None:
    v = None
    for _, r in read_jsonl(session_dir / "events_watchdog.jsonl", []):
        if r.get("event_type") == "run_verdict":
            v = (r.get("payload") or {}).get("verdict")
    return v


def package(story: Story, out_root: Path, video: bool = True) -> Path:
    media_id = story.media_id
    if not media_id or not MEDIA_ID_RE.match(str(media_id)):
        raise ValueError(f"{story.path.name}: media_id must be set (letters, digits, _ . -)")
    sdir = story.session_dir
    sid = sdir.name
    scored = _scored_sessions()
    if sid in scored:
        raise ValueError(f"session {sid} belongs to scored batch {scored[sid]}; paper media use unscored sessions only")

    prep: Prepared = prepare(story)
    out = Path(out_root) / str(media_id)
    if out.exists():
        shutil.rmtree(out)
    (out / "figure").mkdir(parents=True)
    shutil.copy2(story.path, out / "story.yaml")

    from .render_figure import export_frames, frame_times, render_figure
    figs = []
    for width in ("double", "single"):
        figs += render_figure(prep, out / "figure" / f"{media_id}_{width}", width)
    frames_csv = None
    cams = list(story.figure_frame_cameras or [])
    if cams and frame_times(prep):
        frames_csv = export_frames(prep, out / "frames", cams)

    vid = None
    if video:
        from .render_video import render_poster, render_video
        vid = render_video(prep, out / "video" / f"{media_id}.mp4")
        render_poster(prep, out / "video" / f"{media_id}_poster.jpg")

    cfg = {r: json.loads((sdir / f"agent_config_{r}.json").read_text()) for r in ("executor", "watchdog")
           if (sdir / f"agent_config_{r}.json").exists()}
    stage = REPO / "assets" / "usd" / "assembly_room.usd"
    dirty = _git("status", "--porcelain")
    meta = {
        "media_id": str(media_id),
        "kind": story.kind,
        "purpose": "media",
        "purpose_note": "illustration only; unscored session, never feeds a paper number",
        "generated_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "tags": list(story.tags or []),
        "notes": story.notes or "",
        "session": {
            "session_id": sid,
            "in_scored_batch": False,
            "scenario": story.scenario,
            "fault_class": story.fault_class,
            "assembly": story.assembly,
            **_orchestrator(sdir),
            "run_verdict": _verdict(sdir),
            "t0_wall_time_iso": epoch_to_iso(prep.session.t0_epoch),
            "t0_source": prep.session.t0_source,
            "agents": {r: {k: c.get(k) for k in ("model", "effort", "claude_cli_version", "scenario")}
                       for r, c in cfg.items()},
            "injections": _injections(sdir),
        },
        "window_s": [round(prep.timemap.r_start, 3), round(prep.timemap.r_end, 3)],
        "window_wall_time_iso": [epoch_to_iso(prep.session.epoch(prep.timemap.r_start)),
                                 epoch_to_iso(prep.session.epoch(prep.timemap.r_end))],
        "key_frames_s": frame_times(prep),
        "frame_cameras": cams,
        "video_camera": str(story.camera) if video else None,
        "thresholds": prep.thresholds,
        "signals": {k: v.source for k, v in prep.signals.items()},
        "provenance": {
            "git_sha": _git("rev-parse", "HEAD"),
            "git_dirty": bool(dirty),
            "story_sha256": _sha256(story.path),
            "stage_sha256": _sha256(stage),
            "replay_state_sha256": _sha256(sdir / "replay" / "replay_state.npz"),
        },
        "files": sorted(str(p.relative_to(out)) for p in out.rglob("*") if p.is_file()),
    }
    with open(out / "media.yaml", "w", encoding="utf-8") as fh:
        fh.write("# Paper media provenance. See paper_media/outputs/README.md.\n")
        yaml.safe_dump(meta, fh, sort_keys=False, allow_unicode=True, width=110)
    print(f"packaged {media_id} -> {out}")
    return out
