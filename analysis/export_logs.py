"""Public log export: python3 -m analysis.export_logs [--repo .] [--out logs]

Writes <out>/sessions/<session>/ for every session in agents/transcripts/sessions:
the session's own log files (everything except frames/ and bag/) plus both agents'
Claude Code transcripts as transcript_<role>.jsonl.gz, mapped with
analysis.transcripts.map_transcript.  Writes <out>/index.csv, one row per session.

Sanitizing, applied to every file written:
  - transcript attachment records of type DROP_ATTACHMENTS are removed: the operator's
    auto-memory and CLAUDE.md (instructions), account context (session_context,
    credential_org) and Claude Code's own system prompt (prompt_snapshot);
  - REDACT strings are replaced in all text (operator email, home directory).
After writing, every output is re-read and checked for LEAK_PATTERNS; any hit fails.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import re
import shutil
from pathlib import Path

import yaml

from .cli import default_transcript_dir
from .transcripts import index_transcripts, map_transcript

DROP_ATTACHMENTS = {"instructions", "session_context", "credential_org", "prompt_snapshot"}
DROP_KEYS = {"serverClassifierRequest"}
REDACT = [("mnasr3@asu.edu", "<operator email>"), ("/home/mohammad", "~")]
LEAK_PATTERNS = ["mnasr3", "@asu.edu", "/home/mohammad", "# Memory Index", "organizationUuid",
                 "AI Governance Requirements"]
SKIP_DIRS = {"frames", "bag"}
ROLES = ("watchdog", "executor")


def redact(text: str) -> str:
    for old, new in REDACT:
        text = text.replace(old, new)
    return text


def sanitize_transcript(src: Path) -> bytes:
    out = []
    for line in src.open(encoding="utf-8"):
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("type") == "attachment" and (d.get("attachment") or {}).get("type") in DROP_ATTACHMENTS:
            continue
        for k in DROP_KEYS:
            d.pop(k, None)
        out.append(redact(json.dumps(d, ensure_ascii=False)))
    return ("\n".join(out) + "\n").encode("utf-8")


def batch_of(batches_root: Path) -> dict[str, tuple[str, str]]:
    """session id -> (batch id, purpose) from batch.yaml sessions; known_invalid -> purpose 'invalid'.
    A session named only in a comment (e.g. a void run) is in no batch."""
    found: dict[str, tuple[str, str]] = {}
    for b in sorted(batches_root.glob("*/batch.yaml")):
        y = yaml.safe_load(b.read_text(encoding="utf-8")) or {}
        for s in y.get("sessions") or []:
            found.setdefault(str(s.get("session_id")), (b.parent.name, str(y.get("purpose") or "")))
        for s in y.get("known_invalid") or []:
            found[str(s.get("session_id"))] = (b.parent.name, "invalid")
    return found


def media_sessions(stories_root: Path) -> set[str]:
    return {sid for s in stories_root.glob("*.yaml")
            for sid in re.findall(r"session_dir:.*?(\d{8}T\d{6}Z)", s.read_text(encoding="utf-8"))}


def export(repo: Path, out: Path, tdir: Path) -> list[dict]:
    sessions_root = repo / "agents" / "transcripts" / "sessions"
    batches = batch_of(repo / "paper_results" / "batches")
    media = media_sessions(repo / "paper_media" / "stories")
    index = index_transcripts(tdir)
    if (out / "sessions").exists():
        shutil.rmtree(out / "sessions")
    rows = []
    for s in sorted(p for p in sessions_root.iterdir() if p.is_dir()):
        dst = out / "sessions" / s.name
        for f in sorted(s.rglob("*")):
            rel = f.relative_to(s)
            if not f.is_file() or rel.parts[0] in SKIP_DIRS:
                continue
            (dst / rel).parent.mkdir(parents=True, exist_ok=True)
            (dst / rel).write_text(redact(f.read_text(encoding="utf-8")), encoding="utf-8")
        row = {"session": s.name, "batch": "", "purpose": "", "media": s.name in media}
        row["batch"], row["purpose"] = batches.get(s.name, ("", "media" if s.name in media else ""))
        for role in ROLES:
            cfg_path = s / f"agent_config_{role}.json"
            if not cfg_path.exists():
                row[f"{role}_model"] = row[f"{role}_effort"] = row[f"{role}_transcript"] = ""
                continue
            cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
            row["scenario"] = row.get("scenario") or (cfg.get("scenario") or "")
            row["assembly"] = cfg.get("assembly") or ""
            row[f"{role}_model"], row[f"{role}_effort"] = cfg.get("model") or "", cfg.get("effort") or ""
            m = map_transcript(index, role, cfg.get("launched_wall_time_iso"))
            if m.status != "unique":
                raise SystemExit(f"{s.name} {role}: transcript mapping {m.status} {m.candidates}")
            name = f"transcript_{role}.jsonl.gz"
            (dst / name).write_bytes(gzip.compress(sanitize_transcript(m.transcript.path), 9, mtime=0))
            row[f"{role}_transcript"] = f"sessions/{s.name}/{name}"
        rows.append(row)
    cols = ["session", "batch", "purpose", "media", "scenario", "assembly",
            "watchdog_model", "watchdog_effort", "executor_model", "executor_effort",
            "watchdog_transcript", "executor_transcript"]
    with (out / "index.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})
    return rows


def check_leaks(out: Path) -> list[str]:
    hits = []
    for f in sorted(out.rglob("*")):
        if not f.is_file():
            continue
        text = (gzip.decompress(f.read_bytes()) if f.suffix == ".gz" else f.read_bytes()).decode("utf-8", "replace")
        hits += [f"{f.relative_to(out)}: {p}" for p in LEAK_PATTERNS if p in text]
    return hits


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo", type=Path, default=Path("."))
    ap.add_argument("--out", type=Path, default=Path("logs"))
    ap.add_argument("--transcripts", type=Path, default=None, help="Claude project transcript dir")
    a = ap.parse_args()
    rows = export(a.repo, a.out, a.transcripts or default_transcript_dir(a.repo))
    hits = check_leaks(a.out)
    if hits:
        raise SystemExit("leak check failed:\n" + "\n".join(hits[:50]))
    n_t = sum(bool(r.get(f"{role}_transcript")) for r in rows for role in ROLES)
    print(f"{len(rows)} sessions, {n_t} transcripts -> {a.out}; leak check clean")


if __name__ == "__main__":
    main()
