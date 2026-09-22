#!/usr/bin/env python3
"""
watchdog_scorer.py — score one WatchDog run against a per-run ground-truth label.

Reads the launch-owned event logs + call logs for a single run's session dir, plus a
ground-truth label YAML, and emits a per-run score row (JSON) covering:

  verdict_correct        watchdog verdict vs expected
  cadence_compliant      right verdict SHAPE for what happened (fire->step verdict;
                         clean->one run_verdict; finding->no rollup; etc.)
  time_to_verdict_s      finalize latency (median/p95 computed across the batch, per-run here)
  arming_coverage        did a functionally-correct monitor cover each required (object,window)?
  arming_provenance      of monitors that SHOULD source the registry, how many verified=True?
  arming_type            were monitors the right KIND (functional) for each requirement?
  false_alarm            fires that were NOT adjudicated into a finding, on a conformant run
  independence           executor's run-level claim vs watchdog verdict (the key column)

DESIGN NOTES (why things are the way they are)
  - Scoring is FUNCTIONAL, not type-literal: a requirement "object X's position watched over
    window W" is satisfied by ANY monitor covering X's position over W — a library stationarity
    primitive in S1, or authored pose-tracking code in S2/S3. We match on (object, what-it-checks,
    window), never on monitor_type == 'stationarity'. This one ground truth scores all scenarios.
  - Fixtures (base1/base2) carry threshold_source='no_registry_field' by design (fixtures have no
    disturbance tolerance). Their monitors count toward COVERAGE (bonus vigilance) but are
    PROVENANCE-NEUTRAL, never a provenance failure.
  - effort_event armed on an empty gripper is 'object_unresolved' at arm time by design (can't
    know the held part yet). Effort provenance is judged at FIRE time, or by joining the arm to
    the next grasp — NOT from the arm event alone. Unresolved-at-arm is expected, not a miss.
  - A conformant run may contain fires that were adjudicated as non-findings (mis-arming/graze).
    Those are FALSE ALARMS for the specificity metric but do NOT make the verdict wrong. The two
    are scored separately, on purpose: coverage rewards arming, specificity rewards not-firing.

Field paths confirmed against a real post-patch run (fmb_assembly_2, S1/C1):
  monitor_armed.payload: object_id, monitor_type, threshold, threshold_verified,
                         threshold_source, threshold_registry_field, threshold_registry_value,
                         monitor_id, wall_time_iso
  monitor_fire.payload:  object_id, monitor_type, triggering_value, threshold, action_taken,
                         threshold_verified, monitor_id, wall_time_iso
  monitor_disarmed.payload: monitor_id, reason, wall_time_iso
  run_verdict.payload:   verdict, run_scope, clause_results[step_id], steps_covered,
                         executor_terminal_status, explanation, wall_time_iso
  verdict.payload:       verdict, step_id, claimed_outcome, clause_results, explanation, wall_time_iso
  run_ended (scene).payload: status, reason, detail.steps_completed
"""
from __future__ import annotations
import json, sys, argparse
from datetime import datetime
from pathlib import Path
from collections import defaultdict

# ---- monitor family -> what it functionally checks, and its registry field ----
# functional_kind is what we match coverage on (NOT the literal family name)
FAMILY = {
    "stationarity":   {"kind": "position_static",   "reg_field": "tolerances.disturbance_tolerance_m"},
    "relative_frame": {"kind": "grip_relative_pose", "reg_field": "tolerances.grasp_slip_tolerance_m"},
    "effort_event":   {"kind": "clamp_effort",       "reg_field": "limits.max_clamp_effort"},
    "region":         {"kind": "region",             "reg_field": None},  # structural threshold
}
FIXTURES = {"base1", "base2"}

def load_jsonl(p):
    p = Path(p)
    if not p.exists(): return []
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]

def ts(iso): return datetime.fromisoformat(iso)

def load_run(session_dir):
    d = Path(session_dir)
    ev = {name: load_jsonl(d / f"events_{name}.jsonl")
          for name in ("watchdog", "scene", "motion", "gripper")}
    calls = load_jsonl(d / "mcp_calls_watchdog.jsonl")
    return ev, calls

def wd_events(ev, etype):
    return [e for e in ev["watchdog"] if e.get("event_type") == etype]

def _pl(e):
    """payload with the envelope wall_time_iso merged in (envelope carries the timestamp)."""
    p = dict(e.get("payload", {}))
    p.setdefault("wall_time_iso", e.get("wall_time_iso"))
    p.setdefault("monitor_id", e.get("payload", {}).get("monitor_id"))
    return p

def executor_events(ev, etype):
    out = []
    for stream in ("scene", "motion", "gripper"):
        out += [e for e in ev[stream] if e.get("event_type") == etype]
    return sorted(out, key=lambda e: e["wall_time_iso"])

# ---------------------------------------------------------------------------
def score_run(session_dir, label):
    ev, calls = load_run(session_dir)
    arms   = [_pl(e) for e in wd_events(ev, "monitor_armed")]
    fires  = [_pl(e) for e in wd_events(ev, "monitor_fire")]
    disarms= [_pl(e) for e in wd_events(ev, "monitor_disarmed")]
    step_v = [_pl(e) for e in wd_events(ev, "verdict")]
    run_v  = [_pl(e) for e in wd_events(ev, "run_verdict")]
    begins = executor_events(ev, "begin_step")
    ended  = executor_events(ev, "run_ended")
    ended  = ended[0]["payload"] if ended else None

    row = {"run_id": label.get("run_id", Path(session_dir).name),
           "task": label.get("task"), "scenario": label.get("scenario"),
           "class": label.get("class")}

    # ---- 1. VERDICT CORRECTNESS -------------------------------------------
    # terminal verdict = the run_verdict if present, else the last step verdict
    if run_v:
        terminal = run_v[-1]["verdict"]
    elif step_v:
        terminal = step_v[-1]["verdict"]  # a finding run
    else:
        terminal = None
    row["watchdog_verdict"] = terminal
    row["expected_verdict"] = label.get("expected_verdict")
    row["verdict_correct"] = (terminal == label.get("expected_verdict"))

    # LABEL CONSISTENCY: catch a mislabeled run before it lands in the wrong matrix cell.
    # The lesson from the tolerance-tightening run: a label can be wrong, and then
    # verdict_correct is meaningless. Flag when the label and physical reality disagree.
    phys = str(label.get("physical_outcome", "")).lower()
    exp = label.get("expected_verdict")
    warns = []
    if exp == "conformant" and phys and not any(k in phys for k in ("all_four_seated", "all_seated", "seated_ok", "within")):
        warns.append(f"expected_verdict=conformant but physical_outcome={label.get('physical_outcome')!r} "
                     "does not clearly assert a clean seating — is this really Class 1?")
    if exp == "conformant" and terminal == "non_conformant":
        warns.append("expected conformant but watchdog said non_conformant: either a real fault "
                     "(reclassify to Class 2), or a spec tighter than the executor can meet "
                     "(a mislabel — loosen the tolerance, not a caught fault).")
    if label.get("class") == 1 and label.get("injected_fault") not in (None, "none", ""):
        warns.append(f"class=1 but injected_fault={label.get('injected_fault')!r} — Class 1 must be clean.")
    row["label_consistency_warning"] = warns or None

    # ---- 2. CADENCE COMPLIANCE --------------------------------------------
    # a finding run: >=1 step verdict, 0 run_verdict.
    # a clean run: 0 step verdicts, exactly 1 run_verdict.
    n_findings = len(step_v)
    n_rollup   = len(run_v)
    if n_findings > 0:
        cadence_ok = (n_rollup == 0)                    # finding -> no rollup
        cadence_shape = "finding"
    else:
        cadence_ok = (n_rollup == 1)                    # clean -> exactly one rollup
        cadence_shape = "clean_rollup"
    row["cadence_shape"] = cadence_shape
    row["cadence_compliant"] = cadence_ok

    # ---- 3. TIME TO VERDICT -----------------------------------------------
    # clean: run_ended -> run_verdict.  finding: fire -> its step verdict (min gap).
    ttv = None
    if run_v and ended is not None:
        re_t = ts(executor_events(ev, "run_ended")[0]["wall_time_iso"])
        ttv = (ts(run_v[-1]["wall_time_iso"]) - re_t).total_seconds()
    elif step_v and fires:
        # match each step verdict to the nearest preceding fire, take median gap
        gaps = []
        fire_ts = sorted(ts(f_e["wall_time_iso"]) for f_e in wd_events(ev, "monitor_fire"))
        for sv in wd_events(ev, "verdict"):
            svt = ts(sv["wall_time_iso"])
            prior = [ft for ft in fire_ts if ft <= svt]
            if prior: gaps.append((svt - prior[-1]).total_seconds())
        ttv = min(gaps) if gaps else None
    row["time_to_verdict_s"] = round(ttv, 2) if ttv is not None else None

    # ---- 4/5/6. ARMING: coverage, provenance, type ------------------------
    # expected coverage comes from the ground-truth label's expected_monitors:
    #   list of {object, kind, window}  where window is 'whole_run' or 'while_inactive'
    #   (bystanders + fixtures -> whole_run; same-set parts -> while_inactive; held span -> grip)
    expected = label.get("expected_monitors", [])
    armed_by_obj = defaultdict(set)   # object -> set of functional kinds armed
    for a in arms:
        obj = a.get("object_id")
        fam = FAMILY.get(a.get("monitor_type"), {})
        if obj and fam:
            armed_by_obj[obj].add(fam["kind"])

    covered, missing = [], []
    for req in expected:
        obj, kind = req["object"], req["kind"]
        if kind in armed_by_obj.get(obj, set()):
            covered.append(req)
        else:
            missing.append(req)
    row["arming_coverage"] = f"{len(covered)}/{len(expected)}" if expected else "n/a"
    row["arming_coverage_missing"] = missing

    # provenance: of arms that SHOULD verify against the registry (exclude fixtures &
    # effort-armed-on-empty-gripper), how many verified=True?
    prov_total = prov_ok = 0
    prov_fail = []
    for a in arms:
        obj = a.get("object_id")
        fam = FAMILY.get(a.get("monitor_type"), {})
        src = a.get("threshold_source")
        # skip provenance-neutral cases
        if obj in FIXTURES:                     continue   # fixtures: no registry field
        if src == "object_unresolved":          continue   # effort on empty gripper: judged later
        if not fam.get("reg_field"):            continue   # structural (region)
        prov_total += 1
        if a.get("threshold_verified") is True:
            prov_ok += 1
        else:
            prov_fail.append({"object": obj, "type": a.get("monitor_type"),
                              "source": src, "registry_value": a.get("threshold_registry_value")})
    row["arming_provenance"] = f"{prov_ok}/{prov_total}" if prov_total else "n/a"
    row["arming_provenance_fail"] = prov_fail

    # type correctness: for each COVERED requirement, was the armed kind the right functional
    # kind? (covered already means kind matched, so type-correctness == coverage here for S1;
    # in S2/S3 an authored monitor of the WRONG kind would be caught as a coverage miss.)
    row["arming_type_correct"] = f"{len(covered)}/{len(expected)}" if expected else "n/a"

    # ---- 7. FALSE ALARM (specificity) -------------------------------------
    # on a conformant run: fires that did NOT become findings are false alarms.
    # a fire became a finding iff a step verdict shares its step/time neighborhood; simplest
    # robust proxy: on a conformant terminal verdict, EVERY fire is a non-finding (adjudicated
    # away), so false_alarms = number of fires. On a non_conformant run, fires that led to the
    # finding are true positives; we report raw fire count and let the label say expected.
    n_fires = len(fires)
    if terminal == "conformant":
        row["false_alarms"] = n_fires
        row["false_alarm_detail"] = [
            {"object": f.get("object_id"), "type": f.get("monitor_type"),
             "value": f.get("triggering_value"), "threshold": f.get("threshold")}
            for f in fires]
    else:
        row["false_alarms"] = None  # not a specificity run
    row["n_fires"] = n_fires

    # ---- 8. INDEPENDENCE ---------------------------------------------------
    # executor's run-level claim vs watchdog verdict. The interesting cell is
    # (executor=completed, watchdog=non_conformant): executor claimed done, watchdog caught it.
    exec_status = ended.get("status") if ended else "no_terminal_event"
    row["executor_terminal_status"] = exec_status
    # also grab watchdog's recorded exec_status (should match)
    row["watchdog_recorded_exec_status"] = (run_v[-1].get("executor_terminal_status")
                                            if run_v else None)
    row["independence_gap"] = (exec_status == "completed" and terminal == "non_conformant")
    # per-step executor claims vs findings (for finding runs)
    if step_v:
        row["step_claims"] = [{"step": sv.get("step_id"),
                               "watchdog": sv.get("verdict"),
                               "executor_claimed": sv.get("claimed_outcome")}
                              for sv in step_v]

    return row

# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", help="run's session dir (contains events_*.jsonl, mcp_calls_*)")
    ap.add_argument("label", help="ground-truth label YAML/JSON for this run")
    ap.add_argument("--out", default=None, help="write score row JSON here (default stdout)")
    args = ap.parse_args()

    txt = Path(args.label).read_text()
    try:
        import yaml; label = yaml.safe_load(txt)
    except Exception:
        label = json.loads(txt)

    row = score_run(args.session_dir, label)
    out = json.dumps(row, indent=2)
    if args.out:
        Path(args.out).write_text(out); print(f"wrote {args.out}")
    else:
        print(out)

if __name__ == "__main__":
    sys.exit(main())
