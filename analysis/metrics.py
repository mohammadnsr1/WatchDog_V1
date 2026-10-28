"""Assemble every derived per-run column: logs (gates, fires), transcripts (tokens), and the
timeout-vs-coverage check (A3)."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from .config import GATED_SCENARIOS
from .fires import compute_fires
from .gates import compute_gates
from .loader import Loaded, Run
from .sessionlogs import Session, load_session
from .transcripts import Transcript, map_transcript


def _score_json_path(batches_root: Path, run: Run) -> Optional[Path]:
    raw = batches_root / run.batch_id / "raw" / run.session_id
    for name in ("score.json", "score_s4.json"):
        if (raw / name).exists():
            return raw / name
    return None


def timeout_coverage_check(run: Run, batches_root: Path, files: list[Path]) -> None:
    """A3: for each timed-out gate, name the monitors score.json says were NOT covered at that
    step (per_step_coverage[].missed), so the causal link is a named step, not a ratio."""
    d = run.derived
    steps = [x.split("/")[0] for x in (d.get("timeout_step_ids") or "").split(";") if x]
    if not steps:
        d["timeout_step_missed_monitors"] = ""
        return
    p = _score_json_path(batches_root, run)
    if p is None:
        run.set_na("timeout_step_missed_monitors", "no_score_json_in_batch_raw")
        return
    files.append(p)
    sj = json.loads(p.read_text(encoding="utf-8"))
    psc = sj.get("per_step_coverage")
    if not isinstance(psc, list):
        run.set_na("timeout_step_missed_monitors", "score_json_has_no_per_step_coverage")
        return
    out = []
    for st in steps:
        rows = [r for r in psc if r.get("step") == st]
        if not rows:
            out.append(f"{st}:step_not_in_per_step_coverage")
            continue
        r = rows[0]
        missed = ",".join(f"{m.get('object')}/{m.get('monitor')}" for m in (r.get("missed") or []))
        out.append(f"{st}:expected={r.get('expected')},covered={r.get('covered')},missed=[{missed}]")
    d["timeout_step_missed_monitors"] = ";".join(out)


def attach_tokens(run: Run, index: list[Transcript], s: Session) -> None:
    d = run.derived
    roles = ("executor", "watchdog") if run.scenario in GATED_SCENARIOS else ("executor",)
    if run.scenario not in GATED_SCENARIOS:
        for col in ("tokens_in", "tokens_out", "cache_read_tokens", "cache_creation_tokens",
                    "n_assistant_msgs", "transcript", "transcript_model"):
            run.set_na(f"{col}_watchdog", "no_watchdog_role")
        d["transcript_map_status_watchdog"] = "no_watchdog_role"
    d["transcript_map_rule"] = "role_from_mcp_servers+first_record_ts_within[-60,+300]s_of_launched_wall_time_iso"
    for role in roles:
        cfg = s.agent_config.get(role) or {}
        m = map_transcript(index, role, cfg.get("launched_wall_time_iso"))
        d[f"transcript_map_status_{role}"] = m.status
        d[f"transcript_candidates_{role}"] = ";".join(m.candidates)
        if m.status != "unique" or m.transcript is None:
            for col in ("tokens_in", "tokens_out", "cache_read_tokens", "cache_creation_tokens",
                        "n_assistant_msgs", "transcript", "transcript_model"):
                run.set_na(f"{col}_{role}", f"transcript_{m.status}")
            continue
        t = m.transcript
        d[f"tokens_in_{role}"] = t.tokens_in
        d[f"tokens_out_{role}"] = t.tokens_out
        d[f"cache_read_tokens_{role}"] = t.cache_read_tokens
        d[f"cache_creation_tokens_{role}"] = t.cache_creation_tokens
        d[f"n_assistant_msgs_{role}"] = t.n_assistant
        d[f"transcript_{role}"] = t.path.name
        d[f"transcript_model_{role}"] = ";".join(f"{k}:{v}" for k, v in sorted(t.models.items()))
        d[f"transcript_offset_from_launch_s_{role}"] = m.offset_s
        cfg_model = cfg.get("model")
        if cfg_model and t.models and set(t.models) != {cfg_model}:
            run.flags[f"transcript_model_mismatch_{role}"] = f"config={cfg_model},transcript={list(t.models)}"


def enrich_all(loaded: Loaded, sessions_root: Path, batches_root: Path, index: list[Transcript]) -> list[Path]:
    """Compute every derived column for every run.  Returns the list of files read."""
    files: list[Path] = []
    for run in loaded.runs:
        sd = sessions_root / run.session_id
        run.derived["session_dir"] = str(sd)
        run.derived["has_observation_row"] = run.has_observation_row
        run.derived["is_oracle"] = run.is_oracle
        if not sd.exists():
            run.flags["session_dir_missing"] = "1"
            for col in ("run_wall_s", "gate_wait_total_s", "fire_delivery_median_s", "frac_fires_during_motion"):
                run.set_na(col, "session_dir_missing")
            continue
        s = load_session(sd, run.scenario)
        files.extend(s.files_read)
        if run.is_oracle:
            # Oracle: observation reporting only.  Still compute run boundaries for the record.
            compute_gates(run, s)
            compute_fires(run, s)
            for col in ("tokens_in_executor", "tokens_out_executor", "tokens_in_watchdog", "tokens_out_watchdog"):
                run.set_na(col, "oracle_batch_not_an_agent_condition")
            continue
        compute_gates(run, s)
        compute_fires(run, s)
        timeout_coverage_check(run, batches_root, files)
        attach_tokens(run, index, s)
    return files
