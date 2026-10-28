"""Claude Code transcripts -> per-role token usage per run (2e).

Transcripts live in ~/.claude/projects/<project-slug>/<uuid>.jsonl and carry no run or
session id.  Mapping rule (verified unique for all 188 role-sessions in Phase 1):
  role     = from the MCP server names the transcript calls (robot-backend-watchdog ->
             watchdog; motion/scene/gripper/executor-monitor -> executor)
  session  = the transcript whose FIRST record timestamp lies within
             TRANSCRIPT_WINDOW_S of agent_config_<role>.json: launched_wall_time_iso
Anything other than exactly one candidate is reported as unmapped, never guessed.
"""
from __future__ import annotations

import json
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from .config import TRANSCRIPT_ROLE_BY_SERVER, TRANSCRIPT_WINDOW_S


@dataclass
class Transcript:
    path: Path
    first_ts_s: Optional[float]
    last_ts_s: Optional[float]
    role: Optional[str]
    n_assistant: int = 0
    tokens_in: int = 0
    tokens_out: int = 0
    cache_read_tokens: int = 0
    cache_creation_tokens: int = 0
    models: Counter = field(default_factory=Counter)
    servers: Counter = field(default_factory=Counter)
    cli_version: Optional[str] = None


def _ts(s: str) -> float:
    return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()


def read_transcript(path: Path) -> Transcript:
    t = Transcript(path=path, first_ts_s=None, last_ts_s=None, role=None)
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            ts = d.get("timestamp")
            if ts:
                s = _ts(ts)
                t.first_ts_s = s if t.first_ts_s is None else t.first_ts_s
                t.last_ts_s = s
            t.cli_version = t.cli_version or d.get("version")
            if d.get("type") != "assistant":
                continue
            m = d.get("message") or {}
            u = m.get("usage") or {}
            t.n_assistant += 1
            t.tokens_in += int(u.get("input_tokens") or 0)
            t.tokens_out += int(u.get("output_tokens") or 0)
            t.cache_read_tokens += int(u.get("cache_read_input_tokens") or 0)
            t.cache_creation_tokens += int(u.get("cache_creation_input_tokens") or 0)
            if m.get("model"):
                t.models[m["model"]] += 1
            for c in m.get("content") or []:
                if isinstance(c, dict) and c.get("type") == "tool_use":
                    name = str(c.get("name", ""))
                    if name.startswith("mcp__"):
                        t.servers[name.split("__")[1]] += 1
    roles = {TRANSCRIPT_ROLE_BY_SERVER[s] for s in t.servers if s in TRANSCRIPT_ROLE_BY_SERVER}
    if roles == {"watchdog"}:
        t.role = "watchdog"
    elif roles == {"executor"}:
        t.role = "executor"
    elif roles:
        t.role = "mixed"
    return t


def index_transcripts(directory: Path) -> list[Transcript]:
    if not directory.exists():
        return []
    return [read_transcript(p) for p in sorted(directory.glob("*.jsonl"))]


@dataclass
class Mapping:
    status: str                       # unique | none | ambiguous | no_launch_time | no_transcript_dir
    transcript: Optional[Transcript]
    candidates: list[str]
    offset_s: Optional[float]


def map_transcript(index: list[Transcript], role: str, launched_iso: Optional[str]) -> Mapping:
    if not index:
        return Mapping("no_transcript_dir", None, [], None)
    if not launched_iso:
        return Mapping("no_launch_time", None, [], None)
    la = datetime.fromisoformat(launched_iso).timestamp()
    lo, hi = TRANSCRIPT_WINDOW_S
    cands = [t for t in index if t.role == role and t.first_ts_s is not None and lo <= t.first_ts_s - la <= hi]
    if len(cands) == 1:
        return Mapping("unique", cands[0], [cands[0].path.name], cands[0].first_ts_s - la)
    return Mapping("none" if not cands else "ambiguous", None, [c.path.name for c in cands], None)
