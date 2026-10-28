"""CLI: python3 -m analysis [--repo .] [--out out] [--transcripts DIR] [--sessions DIR]

Reads only.  Writes out/per_run.csv, out/table{1,2,3,_cost,_observation}.{csv,tex,na.csv},
out/audit.md, out/manifest.json.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from . import expectations as X
from .config import (ALL_LOADED_BATCHES, CAMPAIGN_BATCHES, EXCLUDED_BATCHES, OBSERVATION_ADJUDICATION_COLUMNS,
                     TABLE_METRIC_COLUMNS, VOIDED_FAILURE_MODE)
from .fractions import Frac
from .loader import Loaded, load_all
from .metrics import enrich_all
from .outputs import (write_audit, write_manifest, write_per_run_csv, write_table_csv, write_table_tex)
from .tables import (Table, build_table1, build_table2, build_table3, build_table_cost, build_table_observation)
from .transcripts import index_transcripts


def default_transcript_dir(repo: Path) -> Path:
    slug = str(repo.resolve()).replace("/", "-")
    return Path.home() / ".claude" / "projects" / slug


def build_everything(repo: Path, transcripts: Path | None = None, sessions: Path | None = None) -> dict[str, Any]:
    """Pure pipeline: load, enrich, build tables.  No writing."""
    batches_root = repo / "paper_results" / "batches"
    sessions_root = sessions or (repo / "agents" / "transcripts" / "sessions")
    tdir = transcripts or default_transcript_dir(repo)
    loaded = load_all(batches_root, ALL_LOADED_BATCHES)
    index = index_transcripts(tdir)
    files = enrich_all(loaded, sessions_root, batches_root, index)
    tables = {
        "table1": build_table1(loaded), "table2": build_table2(loaded), "table3": build_table3(loaded),
        "table_cost": build_table_cost(loaded), "table_observation": build_table_observation(loaded),
    }
    return {"loaded": loaded, "tables": tables, "files": files, "transcript_dir": tdir,
            "n_transcripts_indexed": len(index), "sessions_root": sessions_root, "batches_root": batches_root}


# ---------------------------------------------------------------------------
# audit sections
# ---------------------------------------------------------------------------

def _frac_eq(a: Any, b: Frac) -> bool:
    return isinstance(a, Frac) and a.num == b.num and a.den == b.den


def regression_section(tables: dict[str, Table]) -> tuple[list[str], int]:
    L = ["Provisional hand-computed expectations (analysis/expectations.py) vs pipeline. A disagreement means one side has a bug; nothing is tuned to match.", ""]
    rows, n_bad = [], 0
    t1 = {r["row"]: r for r in tables["table1"].rows}
    t2 = {r["row"]: r for r in tables["table2"].rows}
    for b, exp in X.BASELINE.items():
        for k, col in X.T1_KEYS.items():
            got = t1[b].get(col)
            ok = _frac_eq(got, exp[k])
            n_bad += not ok
            rows.append([f"table1/{b}", col, str(exp[k]), str(got), "ok" if ok else "DISAGREE"])
        got = t2[b].get("step_level_caught")
        ok = _frac_eq(got, exp["c4_caught"])
        n_bad += not ok
        rows.append([f"table2/{b}", "step_level_caught", str(exp["c4_caught"]), str(got), "ok" if ok else "DISAGREE"])
    t3 = {r["row"]: r for r in tables["table3"].rows}
    for b, exp in X.CAPABILITY.items():
        ok = t3[b].get("n") == exp["n"]
        n_bad += not ok
        rows.append([f"table3/{b}", "n", exp["n"], t3[b].get("n"), "ok" if ok else "DISAGREE"])
        for k, col in X.T3_KEYS.items():
            got = t3[b].get(col)
            ok = _frac_eq(got, exp[k])  # type: ignore[arg-type]
            n_bad += not ok
            rows.append([f"table3/{b}", col, str(exp[k]), str(got), "ok" if ok else "DISAGREE"])
    from .outputs import _md_table
    L += _md_table(["cell", "column", "expected", "got", "status"], rows)
    L.append(f"\n**{n_bad} disagreement(s).**")
    return L, n_bad


def voided_section(loaded: Loaded) -> list[str]:
    from .outputs import _md_table
    rows = []
    n_scored = sum(1 for r in loaded.runs if not r.is_oracle)
    voided = []
    for b in CAMPAIGN_BATCHES:
        for ki in loaded.batches[b].known_invalid:
            sid = ki.get("session_id")
            voided.append(sid)
            rows.append([b, sid, VOIDED_FAILURE_MODE.get(sid, "UNCLASSIFIED"), " ".join(str(ki.get("reason", "")).split())[:400]])
    L = [f"**Attempted {n_scored + len(voided)} / scored {n_scored} / voided {len(voided)}** (agent-condition batches; REF excluded; {EXCLUDED_BATCHES} never loaded).",
         "", "Failure modes are hand-classified in analysis/config.py:VOIDED_FAILURE_MODE from the batch.yaml reason text, which is quoted here.", ""]
    L += _md_table(["batch", "session", "failure_mode", "recorded reason"], rows)
    return L


def corrections_section(loaded: Loaded) -> list[str]:
    from .outputs import _md_table
    L = ["Hand-adjustment density per batch (every logged edit resolved to the runs it touched; a batch-wide `*(all rows)*` edit counts once per run).", ""]
    rows = []
    for b in loaded.batches:
        runs = loaded.by_batch(b)
        n = len(runs)
        total = sum(len(r.corrections) for r in runs)
        by_col: Counter = Counter()
        cls: Counter = Counter()
        unspecified = 0
        for r in runs:
            for c in r.corrections:
                if c.column_unspecified:
                    unspecified += 1
                    cls["column_unspecified"] += 1
                for col in c.columns:
                    by_col[col] += 1
                    if col in TABLE_METRIC_COLUMNS and col not in OBSERVATION_ADJUDICATION_COLUMNS:
                        cls["enters_tables(conformance)"] += 1
                    elif col in OBSERVATION_ADJUDICATION_COLUMNS:
                        cls["observation_adjudication"] += 1
                    else:
                        cls["other(" + col + ")"] += 1
        p = loaded.batches[b].corrections
        rows.append([b, n, total, f"{total / n:.2f}" if n else "-", dict(cls), unspecified,
                     p.skipped_template_rows, len(p.malformed_rows), "; ".join(f"{k}:{v}" for k, v in by_col.most_common())])
    L += _md_table(["batch", "runs", "corrections resolved to runs", "per run", "classification", "column '-'", "template rows skipped", "malformed", "by column"], rows)
    L += ["", "Per-run counts:"]
    L += _md_table(["batch", "run", "n corrections", "columns"],
                   [[r.batch_id, r.run_id, len(r.corrections), "; ".join(sorted({c for x in r.corrections for c in x.columns}))]
                    for r in loaded.runs if r.corrections])
    for o in loaded.orphan_observation_rows:
        L.append(f"- orphan observation row skipped: {o}")
    return L


def na_section(loaded: Loaded) -> list[str]:
    from .outputs import _md_table
    by: dict[tuple[str, str], list[str]] = defaultdict(list)
    for r in loaded.runs:
        for col, reason in r.na_reasons.items():
            by[(col, reason)].append(f"{r.batch_id}/{r.run_id}")
    rows = [[col, reason, len(runs), ", ".join(runs) if len(runs) <= 8 else ", ".join(runs[:8]) + f" (+{len(runs) - 8})"]
            for (col, reason), runs in sorted(by.items())]
    return _md_table(["column", "reason", "n runs", "runs"], rows)


def fault_disagreement_section(loaded: Loaded) -> list[str]:
    from .outputs import _md_table
    rows = []
    for r in loaded.runs:
        if r.is_oracle:
            continue
        diff = [(c, r.results.get(f"fault_{c}"), r.results.get(f"scorer_raw_fault_{c}"))
                for c in ("TP", "FP", "FN") if (r.results.get(f"fault_{c}") or "") != (r.results.get(f"scorer_raw_fault_{c}") or "")]
        if diff:
            notes = " / ".join(f"{c.columns}: {c.old_value}->{c.new_value}: {c.reason[:160]}" for c in r.corrections
                               if any(x.startswith("fault_") for x in c.columns))
            rows.append([r.batch_id, r.run_id, "; ".join(f"{c}: adj={a} raw={b}" for c, a, b in diff), notes or "(no fault_* correction logged)"])
    return ["Runs where adjusted fault_* and scorer_raw_fault_* disagree, with the logged correction:", ""] + _md_table(["batch", "run", "disagreement", "corrections note"], rows)


def transcript_section(loaded: Loaded, n_indexed: int, tdir: Path) -> list[str]:
    from .outputs import _md_table
    L = [f"Transcript dir: `{tdir}` ({n_indexed} transcripts indexed).",
         "Rule: role from MCP server names; session = the one transcript whose first record is within [-60, +300] s of agent_config_<role>.json launched_wall_time_iso.", ""]
    c: Counter = Counter()
    fails = []
    for r in loaded.runs:
        for role in ("executor", "watchdog"):
            st = r.derived.get(f"transcript_map_status_{role}")
            if st is None:
                continue
            c[(role, st)] += 1
            if st not in ("unique", "no_watchdog_role"):
                fails.append([r.batch_id, r.run_id, role, st, r.derived.get(f"transcript_candidates_{role}", "")])
    L += _md_table(["role", "status", "n"], [[k[0], k[1], v] for k, v in sorted(c.items())])
    L += ["", f"Unmapped role-sessions: {len(fails)}"]
    if fails:
        L += _md_table(["batch", "run", "role", "status", "candidates"], fails)
    return L


def timeout_section(loaded: Loaded) -> list[str]:
    from .outputs import _md_table
    rows = [[r.batch_id, r.run_id, r.session_id, r.derived.get("timeout_step_ids"), r.results.get("arming_coverage"),
             r.derived.get("timeout_step_missed_monitors") or r.na_reasons.get("timeout_step_missed_monitors", "")]
            for r in loaded.runs if (r.derived.get("n_gate_timeouts") or 0) > 0]
    return ["Gate timeouts (120 s, executor proceeded) against score.json per_step_coverage at the timed-out step (A3). "
            "`missed` names the (object/monitor) pairs the scorer says were NOT armed in time at that step.", ""] + \
        _md_table(["batch", "run", "session", "timed-out gate", "arming_coverage", "coverage at that step"], rows)


def hygiene_section(loaded: Loaded) -> list[str]:
    from .outputs import _md_table
    rows = [[r.batch_id, r.run_id, r.derived.get("n_executor_calls_dropped_contamination"), r.derived.get("executor_contamination_servers"),
             r.derived.get("n_watchdog_calls_dropped_contamination")]
            for r in loaded.runs if (r.derived.get("n_executor_calls_dropped_contamination") or 0) or (r.derived.get("n_watchdog_calls_dropped_contamination") or 0)]
    L = ["MCP call logs are filtered to the role's own servers AND, per server, that server's modal pid. Records dropped:", ""]
    L += _md_table(["batch", "run", "executor records dropped", "foreign servers", "watchdog records dropped"], rows)
    L += ["", "Clock domains: orchestrator wall_ns (epoch), waited_sec (monotonic delta, same process); MCP wall_time_iso = call start, "
          "return = start + elapsed_sec; event envelopes wall_time_iso; fire payload trigger_wall_ns (epoch ns). All on one host. "
          "The monotonic envelope timestamp_ns is never used. Gate request time = grant wall_ns - waited_sec.",
          "", "Delivery asymmetry (D4): S1-S3 delivery is the return of a BLOCKING wait_for_event; S4 delivery is the return of a "
          "get_monitor_fires POLL, so S4 latency is set by when the executor next chose to poll. `delivery_mechanism` carries this per row.",
          "", "Non-comparable columns carried through unchanged: fire_to_next_verdict_* (S1-S3) and fire_to_adjudication_* (S4) are different "
          "quantities; interleaved_monitor_blocks / monitor_calls_in_run / monitor_tool_wall_sec are S4-only by construction.",
          "", "watchdog_postrun_tail_s (run_ended -> last watchdog MCP return) overlaps time_to_verdict_s (= watchdog_adjudication_s); "
          "they describe the same post-run interval from two anchors and must never be summed."]
    flags = [[r.batch_id, r.run_id, json.dumps(r.flags)] for r in loaded.runs if r.flags]
    if flags:
        L += ["", "Per-run flags:"] + _md_table(["batch", "run", "flags"], flags)
    return L


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="analysis")
    ap.add_argument("--repo", type=Path, default=Path("."))
    ap.add_argument("--out", type=Path, default=Path("out"))
    ap.add_argument("--transcripts", type=Path, default=None, help="Claude project transcript dir (default: ~/.claude/projects/<repo slug>)")
    ap.add_argument("--sessions", type=Path, default=None, help="session root (default: <repo>/agents/transcripts/sessions)")
    a = ap.parse_args(argv)
    repo = a.repo.resolve()
    out = a.out
    out.mkdir(parents=True, exist_ok=True)

    r = build_everything(repo, a.transcripts, a.sessions)
    loaded: Loaded = r["loaded"]
    tables: dict[str, Table] = r["tables"]

    outputs: list[Path] = []
    per_run = out / "per_run.csv"
    write_per_run_csv(loaded.runs, per_run)
    outputs.append(per_run)
    captions = {
        "table1": ("Baseline architecture, Sonnet-low, all runs per scenario.", "tab:baseline"),
        "table2": ("Independence: Class 4 (corrupted executor registry).", "tab:independence"),
        "table3": ("Capability: Class 1-2 runs; observation columns suffixed wholebatch cover every class in the batch.", "tab:capability"),
        "table_cost": ("Task-time decomposition. WatchDog blocking cost and post-run adjudication are separate costs.", "tab:cost"),
        "table_observation": ("Observation metrics, whole batch, including the scripted oracle (REF).", "tab:observation"),
    }
    tex_cols = {
        "table1": ["n", "arming_coverage", "grip_coverage", "param_accuracy", "verdict_correct", "fault_recall", "fault_precision",
                   "obs_engagement", "obs_detection", "obs_false_fire_fraction", "false_alarm_rate", "gate_overhead_fraction", "run_wall_median_s"],
        "table2": ["n", "step_level_caught", "verdict_correct", "n_detected_misadjudicated", "n_undetected", "n_other", "fisher_p_one_sided"],
        "table3": ["n", "arming_coverage", "grip_coverage", "param_accuracy", "verdict_correct", "fault_recall", "fault_precision",
                   "obs_engagement_wholebatch", "obs_detection_wholebatch", "obs_false_fire_fraction_wholebatch", "false_alarm_rate",
                   "gate_overhead_fraction", "run_wall_median_s", "frac_fires_during_motion", "frac_fires_during_gripper_server",
                   "fire_delivery_median_s", "tokens_in_executor_sum", "tokens_out_executor_sum", "tokens_in_watchdog_sum", "tokens_out_watchdog_sum"],
        "table_cost": ["n", "total_task_time_s_median", "executor_working_s_median", "watchdog_blocking_cost_s_median",
                       "watchdog_postrun_adjudication_s_median", "gate_overhead_fraction", "gate_overhead_fraction_ex_timeout", "n_gate_timeouts"],
        "table_observation": ["is_oracle", "n_runs", "n_obs_rows_wholebatch", "obs_engagement_wholebatch", "obs_detection_wholebatch", "obs_false_fire_fraction_wholebatch"],
    }
    for name, t in tables.items():
        write_table_csv(t, out / f"{name}.csv")
        write_table_tex(t, out / f"{name}.tex", captions[name][0], captions[name][1], tex_cols[name])
        outputs += [out / f"{name}.csv", out / f"{name}.na.csv", out / f"{name}.tex"]

    reg, n_bad = regression_section(tables)
    sections = {
        "Sessions attempted / scored / voided (A1)": voided_section(loaded),
        "Regression check vs provisional expectations (Phase 6)": reg,
        "Corrections density (D8)": corrections_section(loaded),
        "Gate timeouts vs arming coverage (D2 / A3)": timeout_section(loaded),
        "Adjusted vs scorer-raw fault counts (pitfall 5)": fault_disagreement_section(loaded),
        "Transcript mapping (2e)": transcript_section(loaded, r["n_transcripts_indexed"], r["transcript_dir"]),
        "Log hygiene, clock domains, non-comparable columns": hygiene_section(loaded),
        "Every per-run NA": na_section(loaded),
    }
    audit = out / "audit.md"
    write_audit(audit, loaded, list(tables.values()), sections)
    outputs.append(audit)
    inputs = list(r["files"]) + [p for b in loaded.batches.values() for p in b.files_read]
    tdir: Path = r["transcript_dir"]
    for run in loaded.runs:
        for role in ("executor", "watchdog"):
            name = run.derived.get(f"transcript_{role}")
            if name:
                inputs.append(tdir / name)
    write_manifest(out / "manifest.json", repo, inputs, outputs,
                   {"batches": list(loaded.batches), "n_runs": len(loaded.runs), "regression_disagreements": n_bad})
    print(f"wrote {len(outputs) + 1} files to {out}/ ; runs={len(loaded.runs)} ; regression disagreements={n_bad}")
    return 0
