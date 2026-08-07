#!/usr/bin/env python3
"""Render raw agent transcripts (*_task_*.jsonl) into a readable evaluation view.

NON-INVASIVE: reads the raw transcript only, never writes to it. The raw
*_task_*.jsonl is the full crash-survival record; everything this script
produces is a derived view sitting next to it in the same session dir:

  <agent>_timeline.txt     -- human-readable transcript (the primary artifact)
  <agent>_timeline.jsonl   -- one lean record per event (for programmatic eval)
  <agent>_reasoning.jsonl  -- full verbatim reasoning per turn (sidecar)
  <agent>_results/<turn>_<tool>.json
                           -- full tool_result bodies too big to inline
                              (only written by THIS script; a transcript may
                              already carry its OWN result_ref sidecar under
                              <agent>_task_*_results/, written by the agent
                              harness itself -- that one is read, not written,
                              here)

<agent> is read from the transcript's run_start.agent field -- never
hardcoded, so this works unmodified on both executor and watchdog logs (and
on any future agent that emits the same envelope shapes).

Deterministic and idempotent: no model calls, no network, pure post-
processing of what is already on disk. Re-running regenerates every derived
view from the raw transcript; nothing here is read-modify-write.

Usage:
    python3 render_session_log.py                      # newest session, all transcripts
    python3 render_session_log.py <session_dir>
    python3 render_session_log.py <session_dir>/watchdog_watchdog_task_....jsonl
"""

from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

RESULT_INLINE_CHAR_LIMIT = 500
HEADLINE_RE = re.compile(r"\*\*(.+?)\*\*")
MONITOR_CODE_KEYS = {"code", "predicate", "expression", "source"}


# --------------------------------------------------------------------------
# Small generic helpers
# --------------------------------------------------------------------------

def _wall_ns_to_iso(wall_ns: int) -> str:
    return datetime.fromtimestamp(wall_ns / 1e9, tz=timezone.utc).isoformat()


def _offset_s(wall_ns: int, start_ns: int) -> float:
    return (wall_ns - start_ns) / 1e9


def _json_len(value: Any) -> int:
    try:
        return len(json.dumps(value, default=str))
    except Exception:
        return 0


def _pretty(value: Any) -> str:
    return json.dumps(value, indent=2, default=str, ensure_ascii=False)


def repo_root() -> Path:
    """Walk up from this file for the agents/ + ros2_ws/ markers.

    Deliberately re-implemented rather than importing
    robot_backend.watchdog.session -- this script must run standalone
    without a sourced ROS overlay on PYTHONPATH.
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "agents").is_dir() and (parent / "ros2_ws").is_dir():
            return parent
    return Path(__file__).resolve().parents[1]


def default_sessions_root() -> Path:
    return repo_root() / "agents" / "transcripts" / "sessions"


def find_newest_session(root: Path) -> Path:
    sessions = [p for p in root.iterdir() if p.is_dir()]
    if not sessions:
        raise SystemExit(f"no session directories under {root}")
    return max(sessions, key=lambda p: p.name)


def find_transcripts(session_dir: Path) -> list[Path]:
    return sorted(session_dir.glob("*_task_*.jsonl"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    events = []
    with path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            events.append(json.loads(line))
    return events


# --------------------------------------------------------------------------
# Monitor / predicate code detection (explicit requirement: verbatim, never
# truncated, never sent to a sidecar -- it is a primary evaluation artifact)
# --------------------------------------------------------------------------

def find_monitor_code(arguments: Any, tool_name: str = "") -> list[tuple[str, str]]:
    """Recursively find agent-authored code/predicate/expression/source fields.

    Returns (dotted_key_path, verbatim_text) pairs. Detection is by key name
    (case-insensitive), covering both a custom predicate nested under
    params/clause on arm_monitor and any future write-monitor-style tool that
    carries code directly.

    "source" alone is ambiguous: record_verdict's clause_results legitimately
    carries clause_results[i].source == "instruction" as a provenance tag
    (see _validate_clause_results in watchdog_mcp_server.py), not code. That
    key is only treated as code on a monitor-related tool call (name contains
    "monitor") -- the only place a "source" expression field would plausibly
    mean an agent-authored predicate. "code"/"predicate"/"expression" have no
    such collision and are always treated as code wherever they appear.
    """
    found: list[tuple[str, str]] = []
    allow_source = "monitor" in tool_name.lower()

    def _walk(node: Any, path: str) -> None:
        if isinstance(node, dict):
            for k, v in node.items():
                key_path = f"{path}.{k}" if path else str(k)
                key_lower = k.lower() if isinstance(k, str) else ""
                is_code_key = key_lower in ("code", "predicate", "expression") or (
                    key_lower == "source" and allow_source
                )
                if is_code_key and isinstance(v, str) and v.strip():
                    found.append((key_path, v))
                else:
                    _walk(v, key_path)
        elif isinstance(node, list):
            for i, v in enumerate(node):
                _walk(v, f"{path}[{i}]")

    _walk(arguments, "")
    return found


# --------------------------------------------------------------------------
# Per-tool result summarizers: summarize(arguments, result) -> str
#
# Each takes both the tool_call arguments and the tool_result body, since
# some evaluative fields (threshold, halt_on_fire, clause) live on the call
# and others (monitor_id, reference_values) live on the result.
# --------------------------------------------------------------------------

def _err_or(result: dict[str, Any], ok_fn) -> str:
    if isinstance(result, dict) and result.get("ok") is False:
        return f"ok=False error={result.get('error', result.get('message'))!r}"
    return ok_fn()


def _summ_arm_monitor(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        bits = [
            f"monitor_id={result.get('monitor_id')}",
            f"monitor_type={result.get('monitor_type', arguments.get('monitor_type'))}",
        ]
        params = arguments.get("params")
        if isinstance(params, dict):
            key_params = {
                k: params[k]
                for k in (
                    "threshold", "frames", "max_delta_m", "comparison",
                    "joint_name", "parent_frame", "child_frame",
                )
                if k in params
            }
            if key_params:
                bits.append(
                    "params(" + ", ".join(f"{k}={v}" for k, v in key_params.items()) + ")"
                )
        if "halt_on_fire" in arguments:
            bits.append(f"halt_on_fire={arguments['halt_on_fire']}")
        clause = arguments.get("clause")
        if isinstance(clause, dict) and clause.get("clause_id"):
            bits.append(f"clause={clause['clause_id']}")
        ref = result.get("reference_values")
        if ref:
            bits.append(f"reference_values={json.dumps(ref, default=str)}")
        return "; ".join(bits)

    return _err_or(result, _ok)


def _summ_get_contract(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        contract = result.get("contract") or {}
        n_pre = len(contract.get("preconditions") or [])
        n_eff = len(contract.get("effects") or [])
        n_inv = len(contract.get("invariants") or [])
        return (
            f"object_id={result.get('object_id')} step_id={result.get('step_id')} "
            f"grasp_width_m={contract.get('grasp_width_m')} "
            f"clauses(preconditions={n_pre}, effects={n_eff}, invariants={n_inv})"
        )

    return _err_or(result, _ok)


def _clause_lines(clause_results: list[Any]) -> list[str]:
    lines = []
    for cr in clause_results:
        if not isinstance(cr, dict):
            lines.append(f"  clause <malformed: {cr!r}>")
            continue
        cid = cr.get("clause_id")
        label = cid if cid else f"(instruction:{cr.get('field')})"
        obs = cr.get("observed")
        cmp_ = cr.get("comparison")
        thr = cr.get("threshold")
        passed = cr.get("pass")
        line = f"  clause {label}: observed={obs} {cmp_} {thr} -> pass={passed}"
        note = cr.get("note")
        if note:
            line += f"  note={note}"
        lines.append(line)
    return lines


def _summ_record_verdict(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        outcome = arguments.get("verdict", result.get("verdict"))
        header = f"verdict={outcome} step_id={arguments.get('step_id')}"
        lines = [header]
        lines.extend(_clause_lines(arguments.get("clause_results") or []))
        return "\n".join(lines)

    return _err_or(result, _ok)


def _summ_begin_step(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        bits = [f"step_id={result.get('step_id', arguments.get('step_id'))}"]
        oids = result.get("object_ids", arguments.get("object_ids"))
        if oids is not None:
            bits.append(f"object_ids={oids}")
        gate = result.get("gate")
        if isinstance(gate, dict):
            bits.append(
                "gate(" + ", ".join(f"{k}={v}" for k, v in gate.items()) + ")"
            )
        return " ".join(bits)

    return _err_or(result, _ok)


def _summ_end_step(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        return (
            f"step_id={result.get('step_id', arguments.get('step_id'))} "
            f"claimed_outcome={result.get('claimed_outcome', arguments.get('claimed_outcome'))}"
        )

    return _err_or(result, _ok)


def _channel_counts(evidence: Any, prefix: str = "") -> list[str]:
    """Flatten get_evidence's evidence dict into '<channel[.id]>=<n samples>' pairs."""
    out: list[str] = []
    if isinstance(evidence, list):
        out.append(f"{prefix}={len(evidence)}")
    elif isinstance(evidence, dict):
        for k, v in evidence.items():
            sub = f"{prefix}.{k}" if prefix else str(k)
            out.extend(_channel_counts(v, sub))
    return out


def _summ_query_snapshot(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        objs = result.get("objects") or {}
        frames = result.get("frames") or {}
        bits = [f"objects={len(objs)}", f"frames={len(frames)}"]
        if result.get("joint_state") is not None:
            bits.append("joint_state=present")
        return ", ".join(bits)

    return _err_or(result, _ok)


def _summ_get_evidence(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        start = arguments.get("start_wall_ns")
        end = arguments.get("end_wall_ns")
        bits = []
        if start is not None and end is not None:
            bits.append(f"window=[{start}, {end}]")
        counts = _channel_counts(result.get("evidence"))
        if counts:
            bits.append("samples(" + ", ".join(counts) + ")")
        return "; ".join(bits) if bits else "(empty)"

    return _err_or(result, _ok)


def _summ_capture_frames(arguments: dict[str, Any], result: dict[str, Any]) -> str:
    def _ok() -> str:
        out_dir = result.get("output_dir")
        name = Path(out_dir).name if out_dir else None
        return (
            f"frame_count={result.get('frame_count')} dir={name} "
            f"covered=[{result.get('covered_start_wall_ns')}, {result.get('covered_end_wall_ns')}]"
        )

    def _err() -> str:
        return (
            f"ok=False buffered_range={result.get('buffered_range')} "
            f"subscribed_topics={result.get('subscribed_topics')}"
        )

    if isinstance(result, dict) and result.get("ok") is False:
        return _err()
    return _ok()


def _summ_count_of(key: str):
    def _summ(arguments: dict[str, Any], result: dict[str, Any]) -> str:
        def _ok() -> str:
            items = result.get(key) or []
            return f"{key}={len(items)}"

        return _err_or(result, _ok)

    return _summ


def _summ_generic(arguments: dict[str, Any], result: Any) -> str:
    """Fallback for tools with no dedicated summarizer: ok + top-level scalars,
    arrays reduced to counts, nested dicts reduced to counts."""
    if not isinstance(result, dict):
        return str(result)
    parts = []
    for k, v in result.items():
        if k in ("ok", "command"):
            continue
        if isinstance(v, list):
            parts.append(f"{k}=<list len={len(v)}>")
        elif isinstance(v, dict):
            parts.append(f"{k}=<dict len={len(v)}>")
        else:
            parts.append(f"{k}={v}")
    header = f"ok={result.get('ok')}"
    return header + ("  " + ", ".join(parts) if parts else "")


_SUMMARIZERS: dict[str, Any] = {
    "arm_monitor": _summ_arm_monitor,
    "get_contract": _summ_get_contract,
    "record_verdict": _summ_record_verdict,
    "verdict": _summ_record_verdict,
    "begin_step": _summ_begin_step,
    "end_step": _summ_end_step,
    "query_snapshot": _summ_query_snapshot,
    "get_evidence": _summ_get_evidence,
    "capture_frames": _summ_capture_frames,
    "available_monitor_definitions": _summ_count_of("monitors"),
    "available_event_types": _summ_count_of("event_types"),
}


def summarize_result(tool_name: str, arguments: dict[str, Any], result: Any) -> str:
    fn = _SUMMARIZERS.get(tool_name, _summ_generic)
    try:
        return fn(arguments, result)
    except Exception as exc:  # never let a summarizer crash the render
        return f"<summary failed: {type(exc).__name__}: {exc}>"


# --------------------------------------------------------------------------
# Sidecar loading / writing
# --------------------------------------------------------------------------

def load_full_result(
    session_dir: Path, tool_result_env: dict[str, Any]
) -> tuple[Any, str | None]:
    """Return (full_result_body, existing_ref_note).

    If the raw transcript already externalized this result (result_ref, from
    the agent harness's own inline-size threshold), load it from disk so
    summarization sees the real body, and report its path. If loading fails,
    fall back to the harness's own result_summary.
    """
    if "result" in tool_result_env:
        return tool_result_env["result"], None
    ref = tool_result_env.get("result_ref")
    if ref:
        ref_path = session_dir / ref
        try:
            with ref_path.open("r", encoding="utf-8") as fh:
                return json.load(fh), ref
        except Exception:
            return tool_result_env.get("result_summary"), ref
    return tool_result_env.get("result_summary"), None


def write_own_sidecar(
    session_dir: Path, agent: str, turn: int, tool_name: str, body: Any,
    _seen: dict[str, int],
) -> str:
    """Write body to <agent>_results/<turn>_<tool>.json; return the relative path.

    _seen disambiguates a rare same-turn/same-tool collision within one render
    pass (append _2, _3, ...).
    """
    out_dir = session_dir / f"{agent}_results"
    out_dir.mkdir(parents=True, exist_ok=True)
    base = f"{turn}_{tool_name}"
    _seen[base] = _seen.get(base, 0) + 1
    name = base if _seen[base] == 1 else f"{base}_{_seen[base]}"
    path = out_dir / f"{name}.json"
    with path.open("w", encoding="utf-8") as fh:
        json.dump(body, fh, indent=2, default=str, ensure_ascii=False)
    return f"{agent}_results/{path.name}"


# --------------------------------------------------------------------------
# Event pairing: turn sequential (llm_request, llm_response, [tool_call,
# tool_result]*) into THINK and TOOL render records, in chronological order.
# --------------------------------------------------------------------------

def pair_events(events: list[dict[str, Any]]) -> tuple[dict[str, Any] | None, dict[str, Any] | None, list[dict[str, Any]]]:
    """Return (run_start, run_end, ordered_records).

    Each record is {"kind": "think", "turn", "request", "response"} or
    {"kind": "tool", "turn", "call", "result"}.
    """
    run_start = None
    run_end = None
    records: list[dict[str, Any]] = []

    pending_requests: dict[int, dict[str, Any]] = {}
    pending_calls: list[dict[str, Any]] = []  # FIFO of unmatched tool_call envs

    for env in events:
        etype = env.get("type")
        if etype == "run_start":
            run_start = env
        elif etype == "run_end":
            run_end = env
        elif etype == "llm_request":
            pending_requests[env.get("turn")] = env
        elif etype == "llm_response":
            request = pending_requests.pop(env.get("turn"), None)
            records.append({"kind": "think", "turn": env.get("turn"), "request": request, "response": env})
        elif etype == "tool_call":
            pending_calls.append(env)
        elif etype == "tool_result":
            call = None
            for i, c in enumerate(pending_calls):
                if c.get("turn") == env.get("turn") and c.get("name") == env.get("name"):
                    call = pending_calls.pop(i)
                    break
            records.append({"kind": "tool", "turn": env.get("turn"), "call": call, "result": env})
        # tools_discovered and any unrecognized envelope types are intentionally
        # not rendered in the timeline -- the spec's field list is exhaustive.

    return run_start, run_end, records


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------

def render_think(
    rec: dict[str, Any], start_ns: int
) -> tuple[list[str], dict[str, Any], dict[str, Any]]:
    """Returns (txt_lines, lean_record, reasoning_sidecar_record)."""
    response = rec["response"]
    request = rec["request"]
    turn = rec["turn"]
    t = _offset_s(response["wall_ns"], start_ns)

    latency_s = None
    if request is not None:
        latency_s = _offset_s(response["wall_ns"], request["wall_ns"])

    reasoning = response.get("reasoning") or {}
    summary = reasoning.get("summary")
    headline = None
    if isinstance(summary, str) and summary.strip():
        heads = HEADLINE_RE.findall(summary)
        if heads:
            headline = "; ".join(heads)

    tool_calls = response.get("tool_calls") or []
    if tool_calls:
        next_action = ", ".join(tc.get("name", "?") for tc in tool_calls)
    else:
        next_action = "(final text)"

    lat_str = f"{latency_s:.2f}s" if latency_s is not None else "n/a"
    header = f"[t+{t:8.2f}s] TURN {turn:<3} THINK  llm_latency={lat_str}"
    lines = [header]
    if headline:
        lines.append(f"    headline: {headline}")
    lines.append(f"    stop_reason={response.get('stop_reason')} -> {next_action}")

    lean = {
        "type": "think",
        "turn": turn,
        "t": round(t, 3),
        "llm_latency_s": latency_s,
        "headline": headline,
        "stop_reason": response.get("stop_reason"),
        "next": next_action,
    }
    reasoning_record = {
        "turn": turn,
        "wall_ns": response.get("wall_ns"),
        "reasoning": reasoning,
        "text": response.get("text"),
    }
    return lines, lean, reasoning_record


def render_tool(
    rec: dict[str, Any],
    start_ns: int,
    session_dir: Path,
    agent: str,
    sidecar_seen: dict[str, int],
) -> tuple[list[str], dict[str, Any]]:
    call = rec["call"]
    result_env = rec["result"]
    turn = rec["turn"]
    name = result_env.get("name", call.get("name") if call else "?")
    arguments = call.get("arguments", {}) if call else {}
    wall_ns = call["wall_ns"] if call else result_env["wall_ns"]
    t = _offset_s(wall_ns, start_ns)

    elapsed_ns = result_env.get("elapsed_ns")
    latency_s = elapsed_ns / 1e9 if elapsed_ns is not None else None
    lat_str = f"{latency_s:.2f}s" if latency_s is not None else "n/a"

    lines = [f"[t+{t:8.2f}s] TURN {turn:<3} TOOL   {name}  tool_latency={lat_str}"]

    lines.append("    args:")
    for line in _pretty(arguments).splitlines():
        lines.append(f"      {line}")

    monitor_code = find_monitor_code(arguments, tool_name=name)
    for key_path, code_text in monitor_code:
        lines.append(f"    MONITOR CODE ({key_path}):")
        for line in code_text.splitlines() or [""]:
            lines.append(f"      | {line}")

    full_result, existing_ref = load_full_result(session_dir, result_env)
    summary = summarize_result(name, arguments, full_result if isinstance(full_result, dict) else {"ok": None, "value": full_result})

    sidecar_ref = existing_ref
    if existing_ref is None and _json_len(full_result) > RESULT_INLINE_CHAR_LIMIT:
        sidecar_ref = write_own_sidecar(session_dir, agent, turn, name, full_result, sidecar_seen)

    lines.append("    result:")
    for line in summary.splitlines():
        lines.append(f"      {line}")
    if sidecar_ref:
        lines.append(f"      (full body: {sidecar_ref})")

    lean = {
        "type": "tool",
        "turn": turn,
        "t": round(t, 3),
        "tool": name,
        "arguments": arguments,
        "tool_latency_s": latency_s,
        "result_summary": summary,
        "result_ref": sidecar_ref,
    }
    if monitor_code:
        lean["monitor_code"] = [{"path": p, "code": c} for p, c in monitor_code]

    return lines, lean


def render_transcript(transcript_path: Path, session_dir: Path) -> None:
    events = read_jsonl(transcript_path)
    run_start, run_end, records = pair_events(events)

    if run_start is None:
        print(f"skip {transcript_path.name}: no run_start envelope")
        return

    agent = run_start.get("agent") or transcript_path.stem.split("_")[0]
    start_ns = run_start["wall_ns"]

    txt_lines: list[str] = []
    lean_records: list[dict[str, Any]] = []
    reasoning_records: list[dict[str, Any]] = []
    sidecar_seen: dict[str, int] = {}

    txt_lines.append(f"agent={agent}")
    txt_lines.append(f"provider={run_start.get('provider')}  model={run_start.get('model')}")
    txt_lines.append(f"task_name={run_start.get('task_name')}")
    txt_lines.append(f"system_prompt_sha256={run_start.get('system_prompt_sha256')}")
    txt_lines.append(f"start={_wall_ns_to_iso(start_ns)}")
    txt_lines.append("-" * 78)

    llm_time_total = 0.0
    tool_time_total = 0.0
    wait_time_total = 0.0

    for rec in records:
        if rec["kind"] == "think":
            lines, lean, reasoning_rec = render_think(rec, start_ns)
            if lean["llm_latency_s"] is not None:
                llm_time_total += lean["llm_latency_s"]
            reasoning_records.append(reasoning_rec)
        else:
            lines, lean = render_tool(rec, start_ns, session_dir, agent, sidecar_seen)
            if lean["tool_latency_s"] is not None:
                tool_time_total += lean["tool_latency_s"]
                if lean["tool"] == "wait_for_event":
                    wait_time_total += lean["tool_latency_s"]

        txt_lines.extend(lines)
        txt_lines.append("")
        lean_records.append(lean)

    txt_lines.append("-" * 78)
    if run_end is not None:
        wall_total = _offset_s(run_end["wall_ns"], start_ns)
        txt_lines.append(
            f"termination_reason={run_end.get('termination_reason')}  "
            f"turn_count={run_end.get('turn_count')}  "
            f"verdict_count={run_end.get('verdict_count')}"
        )
    else:
        wall_total = _offset_s(events[-1]["wall_ns"], start_ns) if events else 0.0
        txt_lines.append("(no run_end envelope -- transcript may be from a run in progress)")

    active_tool_time = tool_time_total - wait_time_total
    txt_lines.append(
        "LATENCY SUMMARY: "
        f"llm_time_total={llm_time_total:.2f}s  "
        f"tool_time_total={tool_time_total:.2f}s "
        f"(of which wait_for_event={wait_time_total:.2f}s, other_tools={active_tool_time:.2f}s)  "
        f"wall_total={wall_total:.2f}s"
    )

    # --- write outputs ---
    (session_dir / f"{agent}_timeline.txt").write_text(
        "\n".join(txt_lines) + "\n", encoding="utf-8"
    )
    with (session_dir / f"{agent}_timeline.jsonl").open("w", encoding="utf-8") as fh:
        for lean in lean_records:
            fh.write(json.dumps(lean, default=str, ensure_ascii=False) + "\n")
    with (session_dir / f"{agent}_reasoning.jsonl").open("w", encoding="utf-8") as fh:
        for rr in reasoning_records:
            fh.write(json.dumps(rr, default=str, ensure_ascii=False) + "\n")

    print(f"rendered {agent}: {transcript_path.name}")
    print(f"  -> {agent}_timeline.txt, {agent}_timeline.jsonl, {agent}_reasoning.jsonl")
    print(
        f"  latency: llm={llm_time_total:.2f}s  tool={tool_time_total:.2f}s "
        f"(wait={wait_time_total:.2f}s)  wall={wall_total:.2f}s"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "session",
        nargs="?",
        default=None,
        help="session directory or a specific *_task_*.jsonl file "
        "(default: newest session under agents/transcripts/sessions)",
    )
    args = ap.parse_args()

    if args.session is None:
        session_dir = find_newest_session(default_sessions_root())
        transcripts = find_transcripts(session_dir)
    else:
        target = Path(args.session).expanduser().resolve()
        if target.is_file():
            session_dir = target.parent
            transcripts = [target]
        elif target.is_dir():
            session_dir = target
            transcripts = find_transcripts(target)
        else:
            raise SystemExit(f"{target} not found")

    if not transcripts:
        raise SystemExit(f"no *_task_*.jsonl transcripts found in {session_dir}")

    print(f"session: {session_dir}")
    for path in transcripts:
        render_transcript(path, session_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
