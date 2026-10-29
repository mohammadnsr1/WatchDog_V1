"""Web assets for the Run Explorer: <run_id>.mp4, <run_id>_poster.jpg, <run_id>.json, manifest.json.

Only trimmed event fields leave the code repository: t_real, t_video, lane,
kind, label, detail. No prompts beyond the story's hand-picked excerpts (which
are not exported here), no file system paths, no raw records.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

from .load_session import load_story, run_metadata
from .render_video import render_poster, render_video
from .timeline import prepare

MAX_MP4_BYTES = 90 * 1024 * 1024
DETAIL_CHARS = 280
WEB_KINDS_SKIP = {"skill_start", "skill_end", "phase", "ready", "gate_open", "orchestrator_start",
                  "gate_unguarded_by_config", "tool_call"}
_PATH_RX = re.compile(r"(?:/home|/tmp|/usr|/opt|/root|~)/[^\s'\",)]+")
_SAFE_ID = re.compile(r"^[A-Za-z0-9_.-]+$")


class SiteError(RuntimeError):
    pass


def scrub(text: str, n: int = DETAIL_CHARS) -> str:
    s = _PATH_RX.sub("[path]", " ".join(str(text).split()))
    return s if len(s) <= n else s[: n - 3].rstrip() + "..."


def check_site(site: Path) -> Path:
    site = Path(site).expanduser().resolve()
    if not (site / "index.html").is_file():
        raise SiteError(f"{site} has no index.html; refusing to write (is --site the website repository root?)")
    return site


def run_json(prep, run_id: str, meta: dict) -> dict:
    tm = prep.timemap
    events = []
    for e in prep.events:
        if e.kind in WEB_KINDS_SKIP:
            continue
        events.append({
            "t_real": round(e.t, 3),
            "t_video": round(tm.real_to_video(e.t), 3),
            "lane": e.lane,
            "kind": e.kind,
            "label": scrub(e.label, 90),
            "detail": scrub(e.detail),
        })
    return {
        "run_id": run_id,
        "title": scrub(prep.story.title or run_id, 120),
        "subtitle": scrub(prep.story.subtitle or "", 200),
        "scenario": meta.get("scenario"),
        "fault_class": meta.get("fault_class"),
        "assembly": meta.get("assembly"),
        "trial": meta.get("trial"),
        "video_duration_s": round(tm.duration, 3),
        "real_window_s": [round(tm.r_start, 3), round(tm.r_end, 3)],
        "events": events,
    }


def export_web(story_paths: list[Path], site: Path) -> None:
    site = check_site(site)
    runs_dir = site / "static" / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    man_path = runs_dir / "manifest.json"
    try:
        manifest = json.loads(man_path.read_text(encoding="utf-8")) if man_path.exists() else {"runs": []}
    except json.JSONDecodeError:
        print(f"warning: {man_path} is not valid JSON; starting a new manifest", file=sys.stderr)
        manifest = {"runs": []}
    by_id = {r.get("run_id"): r for r in manifest.get("runs", [])}

    for sp in story_paths:
        story = load_story(sp)
        prep = prepare(story)
        run_id = str(story.run_id or prep.session.dir.name)
        if not _SAFE_ID.match(run_id):
            raise SiteError(f"run_id {run_id!r} must match {_SAFE_ID.pattern}")
        meta = run_metadata(prep.session, story)
        mp4 = runs_dir / f"{run_id}.mp4"
        tmp = runs_dir / f".{run_id}.partial.mp4"   # ffmpeg picks the container from the extension
        render_video(prep, tmp)
        size = tmp.stat().st_size
        video_ok = size <= MAX_MP4_BYTES
        if video_ok:
            tmp.replace(mp4)
        else:
            tmp.unlink()
            print(f"warning: {run_id}.mp4 is {size / 2**20:.1f} MB (> 90 MB, GitHub rejects > 100 MB); skipped. "
                  f"Raise speed.idle or crop window in {sp}.", file=sys.stderr)
        poster = runs_dir / f"{run_id}_poster.jpg"
        render_poster(prep, poster)
        data = run_json(prep, run_id, meta)
        (runs_dir / f"{run_id}.json").write_text(json.dumps(data, indent=1), encoding="utf-8")
        by_id[run_id] = {
            "run_id": run_id,
            "title": data["title"],
            "subtitle": data["subtitle"],
            "scenario": data["scenario"],
            "fault_class": data["fault_class"],
            "assembly": data["assembly"],
            "trial": data["trial"],
            "video": f"static/runs/{run_id}.mp4" if video_ok else None,
            "poster": f"static/runs/{run_id}_poster.jpg",
            "json": f"static/runs/{run_id}.json",
        }
        print(f"exported {run_id}: {len(data['events'])} events, video {'ok' if video_ok else 'SKIPPED'}")

    manifest["runs"] = sorted(by_id.values(), key=lambda r: (str(r.get("scenario")), str(r.get("fault_class")),
                                                             str(r.get("trial")), r["run_id"]))
    man_path.write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    print(f"updated {man_path} ({len(manifest['runs'])} runs)")
