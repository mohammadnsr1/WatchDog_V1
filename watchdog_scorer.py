#!/usr/bin/env python3
"""
watchdog_scorer.py — score one WatchDog run against a per-run ground-truth label.

FINALIZED for the always-final-verdict cadence: every run emits exactly one final run_verdict,
plus zero or more step-scoped findings. Scores verdict, cadence, detection (TP/FP/TN/FN),
arming (coverage/provenance/type with WINDOW TIMING enforced), latency, and independence.

USAGE:  python3 watchdog_scorer.py <session_dir> <label.yaml> [--out row.json]

TERMINAL MODEL (post-cadence-change):
  - Final verdict is ALWAYS present: run_verdict.verdict in {conformant, non_conformant,
    incomplete}. This is the END-STATE assessment.
  - Step findings (event_type 'verdict', verdict != conformant/valid_variation) are the TIMELINE
    of what went wrong and when.
  - recovered = bool(findings) and final verdict conformant  (violation caught, then corrected).
  - The (final_verdict, recovered) PAIR distinguishes: clean (conformant,False),
    recovery (conformant,True), broken (non_conformant,-), aborted (incomplete,-).

DETECTION SCORING (the confusion matrix the paper's claim lives in):
  The label states, per run, whether a REAL violation occurred and where (violation_steps).
  Each watchdog finding is classified TP (matched a real violation) or FP (flagged where none was).
  Each real violation with no matching finding is FN (missed). Clean steps with no finding are TN.
  This is what turns "the watchdog said X" into "the watchdog correctly detects real violations at
  rate R with false-alarm rate F" — recall/precision/specificity.

LABEL SCHEMA (per run):
  run_id, task, scenario, class
  expected_final_verdict          conformant | non_conformant | incomplete
  expected_terminal_status        completed | aborted | no_terminal_event
  fault_origin                    none | injected | noise_induced  (Class 1 -> none)
  violation_occurred              bool — did a REAL violation happen this run (ground truth)
  violation_steps                 list of {step, kind, object, note} — where/what, for TP matching
                                  (empty when violation_occurred is false)
  expected_recovered              bool — did the run recover (violation then conformant end-state)
  expected_monitors               [{object, kind, window}]  (coverage ground truth)
  physical_outcome, executor_claimed, notes   (recorded observations)

FIELD PATHS (confirmed against post-change fmb_assembly_2 run):
  monitor_armed.payload: object_id, monitor_type, threshold, threshold_verified, threshold_source,
                         monitor_id ; envelope: wall_time_iso
  monitor_fire.payload:  object_id, monitor_type, triggering_value, threshold, threshold_verified,
                         action_taken, monitor_id ; envelope: wall_time_iso
  verdict.payload:       verdict, step_id, claimed_outcome, clause_results, explanation
  run_verdict.payload:   verdict, run_scope, recovered, findings_during_run, clause_results
                         (each step_id), steps_covered, executor_terminal_status, explanation
  scene run_ended.payload: status, reason
  phase (scene/motion/gripper).payload: phase=='grasped', step_id  (grasp-confirmed time)
"""
from __future__ import annotations
import json, sys, argparse
from datetime import datetime
from pathlib import Path
from collections import defaultdict

FAMILY = {
    "stationarity":   {"kind": "position_static",   "reg_field": "tolerances.disturbance_tolerance_m"},
    "relative_frame": {"kind": "grip_relative_pose", "reg_field": "tolerances.grasp_slip_tolerance_m"},
    "effort_event":   {"kind": "clamp_effort",       "reg_field": "limits.max_clamp_effort"},
    "region":         {"kind": "region",             "reg_field": None},
}
FIXTURES = {"base1", "base2"}
FINDING_VERDICTS = {"non_conformant", "inconclusive"}   # a step verdict that IS a finding

def load_jsonl(p):
    p = Path(p)
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()] if p.exists() else []

def ts(iso): return datetime.fromisoformat(iso)

def load_label(path):
    txt = Path(path).read_text()
    try:
        import yaml
    except ImportError:
        sys.exit("ERROR: PyYAML not installed. Run: pip install pyyaml   (labels are YAML)")
    try:
        return yaml.safe_load(txt)
    except Exception as e:
        sys.exit(f"ERROR parsing label YAML {path}: {e}")

def load_run(session_dir):
    d = Path(session_dir)
    ev = {n: load_jsonl(d / f"events_{n}.jsonl") for n in ("watchdog", "scene", "motion", "gripper")}
    calls = load_jsonl(d / "mcp_calls_watchdog.jsonl")
    return ev, calls

def wd_events(ev, etype): return [e for e in ev["watchdog"] if e.get("event_type") == etype]
def _pl(e):
    p = dict(e.get("payload", {})); p.setdefault("wall_time_iso", e.get("wall_time_iso")); return p

def _obj(p):
    """resolve a monitor's subject object: top-level object_id, else clause.object_id.
    stationarity/relative_frame carry it top-level; effort_event (armed empty-gripper) carries it
    in the clause. Both are the intended subject — read whichever is present."""
    o = p.get("object_id")
    if o and o not in (None, "None"):
        return o
    cl = p.get("clause") or {}
    return cl.get("object_id")
def exec_events(ev, etype):
    out = []
    for s in ("scene", "motion", "gripper"): out += [e for e in ev[s] if e.get("event_type") == etype]
    return sorted(out, key=lambda e: e["wall_time_iso"])

# ---------------------------------------------------------------------------
def score_run(session_dir, label):
    ev, calls = load_run(session_dir)
    arms  = [_pl(e) for e in wd_events(ev, "monitor_armed")]
    fires = [_pl(e) for e in wd_events(ev, "monitor_fire")]
    step_v= [_pl(e) for e in wd_events(ev, "verdict")]
    run_v = [_pl(e) for e in wd_events(ev, "run_verdict")]
    begins= exec_events(ev, "begin_step")
    ended = exec_events(ev, "run_ended")
    ended = _pl(ended[0]) if ended else None

    row = {"run_id": label.get("run_id", Path(session_dir).name),
           "task": label.get("task"), "scenario": label.get("scenario"), "class": label.get("class")}

    # ---- final verdict (ALWAYS present now) --------------------------------
    final = run_v[-1] if run_v else None
    row["watchdog_final_verdict"] = final["verdict"] if final else None
    row["watchdog_recovered"]     = final.get("recovered") if final else None
    row["findings_during_run"]    = final.get("findings_during_run") if final else None
    row["n_run_verdicts"]         = len(run_v)

    exp_fv = label.get("expected_final_verdict", label.get("expected_verdict"))
    row["expected_final_verdict"] = exp_fv
    row["verdict_correct"] = (row["watchdog_final_verdict"] == exp_fv)

    # recovered pair check
    exp_rec = label.get("expected_recovered")
    if exp_rec is not None:
        row["recovered_correct"] = (bool(row["watchdog_recovered"]) == bool(exp_rec))

    # ---- cadence: exactly one final run_verdict, plus 0+ step findings -----
    row["cadence_compliant"] = (len(run_v) == 1)
    if len(run_v) != 1:
        row["cadence_error"] = f"expected exactly 1 run_verdict, found {len(run_v)}"

    # ---- LABEL CONSISTENCY -------------------------------------------------
    warns = []
    phys = str(label.get("physical_outcome", "")).lower()
    if exp_fv == "conformant" and label.get("violation_occurred") and not label.get("expected_recovered"):
        warns.append("expected_final_verdict=conformant with violation_occurred=true but "
                     "expected_recovered not set — a violation that ends conformant IS a recovery; "
                     "set expected_recovered: true, or the verdict should be non_conformant.")
    if label.get("class") == 1 and label.get("violation_occurred"):
        warns.append("class=1 but violation_occurred=true — a run with a real violation is not "
                     "Class 1. Reclassify (Class 2 / recovery), or correct the label.")
    if label.get("class") == 1 and label.get("fault_origin") not in (None, "none", ""):
        warns.append(f"class=1 but fault_origin={label.get('fault_origin')!r} — Class 1 is clean.")
    row["label_consistency_warning"] = warns or None

    # ---- DETECTION: TP / FP / FN, matched per (STEP, OBJECT) --------------
    # A real violation is a (step, object) pair from the label. A watchdog finding is matched to a
    # real violation when BOTH the step AND the object agree. Object for a finding comes from the
    # step verdict's clause/object, else from a monitor fire in that step's window, else the
    # verdict's own object field. This credits each distinct disturbed object as its own TP
    # (e.g. u_orange slip + base1 + hex_blue bystander at one step = 3 TP, not 1).
    real_viol = label.get("violation_steps", []) or []
    real_pairs = {(v.get("step"), v.get("object")) for v in real_viol if v.get("step") and v.get("object")}

    # collect finding (step, object) pairs. A step verdict names the step; its object we take from
    # the verdict payload's clause.object_id if present, else we attribute via fires in that step.
    # Since multiple objects can be flagged at one step (multiple fires), gather ALL fired objects
    # within each finding step's time window.
    # build step windows
    bs_win = sorted([(_pl(b)["wall_time_iso"], b["payload"].get("step_id")) for b in begins])
    def step_window(sid):
        for i,(t,s) in enumerate(bs_win):
            if s == sid:
                t0 = ts(t); t1 = ts(bs_win[i+1][0]) if i+1 < len(bs_win) else None
                return t0, t1
        return None, None
    fires_ev = wd_events(ev, "monitor_fire")
    finding_steps = {sv.get("step_id") for sv in step_v if sv.get("verdict") in FINDING_VERDICTS}
    finding_pairs = set()
    for sid in finding_steps:
        t0, t1 = step_window(sid)
        # objects fired within this step's window
        objs = set()
        for e in fires_ev:
            ft = ts(e["wall_time_iso"])
            if t0 is not None and ft >= t0 and (t1 is None or ft <= t1):
                o = _obj(_pl(e))
                if o: objs.add(o)
        # also the verdict's own clause object, if any
        for sv in step_v:
            if sv.get("step_id") == sid and sv.get("verdict") in FINDING_VERDICTS:
                co = (sv.get("clause") or {}).get("object_id")
                if co: objs.add(co)
        if not objs:
            objs = {None}   # a finding with no resolvable object still counts as a finding at that step
        for o in objs:
            finding_pairs.add((sid, o))

    def pretty(p): return [f"{s}:{o}" for (s,o) in sorted(p, key=lambda x:(x[0] or '', x[1] or ''))]
    tp = real_pairs & finding_pairs
    fn = real_pairs - finding_pairs
    # false positives: finding pairs that don't match a real violation. A finding whose object is
    # None but whose step IS a real-violation step is treated as matching (can't disprove it).
    fp = set()
    for (s,o) in finding_pairs:
        if (s,o) in real_pairs: continue
        if o is None and any(rs == s for (rs,ro) in real_pairs): continue
        fp.add((s,o))
    row["detection"] = {
        "true_positives":  pretty(tp),
        "false_negatives": pretty(fn),
        "false_positives": pretty(fp),
        "n_real_violations": len(real_pairs),
        "n_findings": len(finding_pairs),
    }
    row["detection_recall"]    = (len(tp) / len(real_pairs)) if real_pairs else None
    row["detection_precision"] = (len(tp) / len(finding_pairs)) if finding_pairs else None

    # fires that never became a finding, on a run with no real violation -> monitor false alarms
    # (specificity at the monitor level, separate from verdict-level detection above)
    # An authored check that returns {"value": 0.0} fires (0.0 is a finite value). Agents use
    # that as a throwaway probe to read ctx, and the monitor is disarmed immediately with no
    # verdict. A probe fire is not a false alarm: a real authored violation carries a positive
    # magnitude. Counted separately so probing is still visible in the row.
    def _is_probe(f):
        return (f.get("origin", "library") == "authored"
                and float(f.get("triggering_value") or 0.0) == 0.0)
    real_fires  = [f for f in fires if not _is_probe(f)]
    probe_fires = [f for f in fires if _is_probe(f)]
    n_fires = len(fires)
    row["n_fires"] = n_fires                     # total, probes included
    row["authored_probe_fires"] = len(probe_fires)
    if not label.get("violation_occurred"):
        row["monitor_false_alarms"] = len(real_fires)   # a REAL fire on a clean run is a false alarm
        row["monitor_false_alarm_detail"] = [
            {"object": f.get("object_id"), "type": f.get("monitor_type"),
             "value": f.get("triggering_value"), "threshold": f.get("threshold")} for f in real_fires]
    else:
        row["monitor_false_alarms"] = None      # fires may be true detections; see detection block

    # ---- TIME TO VERDICT ---------------------------------------------------
    ttv = None
    if final and ended:
        ttv = (ts(final["wall_time_iso"]) - ts(exec_events(ev, "run_ended")[0]["wall_time_iso"])).total_seconds()
    row["time_to_verdict_s"] = round(ttv, 2) if ttv is not None else None
    # per-finding latency (fire -> step verdict), for finding runs
    if finding_steps and fires:
        fire_ts = sorted(ts(_pl(e)["wall_time_iso"]) for e in wd_events(ev, "monitor_fire"))
        gaps = []
        for e in wd_events(ev, "verdict"):
            vt = ts(_pl(e)["wall_time_iso"]); prior = [t for t in fire_ts if t <= vt]
            if prior: gaps.append(round((vt - prior[-1]).total_seconds(), 2))
        row["fire_to_finding_s"] = gaps or None

    # ---- ARMING: coverage (WINDOW-TIMED), provenance, type -----------------
    expected = label.get("expected_monitors", [])
    # ===== PER-STEP ARMING COVERAGE (timing-aware) =====
    # Build monitor LIVENESS intervals by pairing arm->disarm on monitor_id. A monitor with no
    # disarm is live to run end. Each monitor knows its object (via _obj) and family.
    end_all = None
    for e in ev["watchdog"]:
        t = ts(e["wall_time_iso"])
        end_all = t if end_all is None or t > end_all else end_all
    # Build a per-monitor_id time-ordered list of arm and disarm events, then pair each arm with
    # the NEXT disarm of the same id. A monitor_id can be armed->disarmed->armed again (re-arm on
    # re-seat), producing MULTIPLE intervals — do not overwrite.
    events_by_id = defaultdict(list)   # mid -> [(time, "arm"/"disarm", obj, mtype)]
    for e in wd_events(ev, "monitor_armed"):
        p = _pl(e); events_by_id[p.get("monitor_id")].append((ts(p["wall_time_iso"]), "arm", _obj(p), p.get("monitor_type")))
    for e in wd_events(ev, "monitor_disarmed"):
        p = _pl(e); events_by_id[p.get("monitor_id")].append((ts(p["wall_time_iso"]), "disarm", None, None))
    # object -> family_kind -> list of (start,end) live intervals
    live = defaultdict(lambda: defaultdict(list))
    for mid, evs in events_by_id.items():
        evs.sort(key=lambda x: x[0])
        open_arm = None  # (t0, obj, mtype)
        for (t, kind, o, mtype) in evs:
            if kind == "arm":
                if open_arm is not None:
                    # arm without an intervening disarm -> close previous at this arm (defensive)
                    pt0, po, pm = open_arm; fam = FAMILY.get(pm)
                    if fam and po: live[po][fam["kind"]].append((pt0, t))
                open_arm = (t, o, mtype)
            else:  # disarm
                if open_arm is not None:
                    pt0, po, pm = open_arm; fam = FAMILY.get(pm)
                    if fam and po: live[po][fam["kind"]].append((pt0, t))
                    open_arm = None
        if open_arm is not None:  # never disarmed -> live to run end
            pt0, po, pm = open_arm; fam = FAMILY.get(pm)
            if fam and po: live[po][fam["kind"]].append((pt0, end_all))
    def is_live(o, kind, w0, w1, grace):
        """monitor of `kind` on `o` live for the BULK of [w0,w1] (grace at each boundary)."""
        need0, need1 = w0 + grace, w1 - grace
        if need1 <= need0: need0, need1 = w0, w1   # step shorter than 2*grace: require any overlap
        for (t0, t1) in live.get(o, {}).get(kind, []):
            if t0 <= need0 and t1 >= need1:
                return True
        return False

    # per-step "released to act" time = watchdog_ready with scope=step for that step_id.
    # The executor blocks at begin_step until this fires (RequestGo/SignalReady gate), so the
    # step's ACTIVE (exposure) window starts here, NOT at begin_step. Falls back to begin_step
    # if no scope=step ready is found for a step.
    ready_step = {}
    # grasped-phase gate: the executor blocks after grasp-confirm until the watchdog signals ready
    # for phase=grasped, so the held-phase monitors (slip/effort) may legitimately be armed during
    # that pause. The arming deadline for those is the GATE RELEASE, not the grasp-confirm event.
    ready_grasp = {}
    for e in wd_events(ev, "watchdog_ready"):
        p = _pl(e)
        if p.get("scope") == "step" and p.get("step_id"):
            ready_step.setdefault(p["step_id"], ts(p["wall_time_iso"]))
        if str(p.get("phase", "")).lower() == "grasped" and p.get("step_id"):
            ready_grasp.setdefault(p["step_id"], ts(p["wall_time_iso"]))
    bs = sorted([( _pl(b)["wall_time_iso"], b["payload"].get("step_id"),
                   (b["payload"].get("object_ids") or [None])[0] )
                 for b in begins], key=lambda x: x[0])
    steps_tl = []
    for i, (t, sid, active) in enumerate(bs):
        # window start = when the executor was RELEASED to act on this step
        w0 = ready_step.get(sid, ts(t))
        # window end = when the NEXT step was released (or run end)
        if i+1 < len(bs):
            nxt_sid = bs[i+1][1]
            w1 = ready_step.get(nxt_sid, ts(bs[i+1][0]))
        else:
            w1 = end_all
        if w1 < w0: w1 = ts(t) if i+1 >= len(bs) else ts(bs[i+1][0])  # safety
        steps_tl.append((sid, active, w0, w1))

    GRACE = __import__("datetime").timedelta(seconds=float(label.get("boundary_grace_s", 3.0)))

    # all objects that must be position-monitored: every FMB object except fixtures are graspable;
    # bystanders + this-set parts. Derive the full object universe from expected_monitors' objects
    # plus the active objects seen, so we don't hardcode set membership.
    all_objs = set()
    for req in expected: all_objs.add(req["object"])
    for _, active, _, _ in steps_tl:
        if active: all_objs.add(active)
    # which objects get slip/effort (the ones the plan grasps) = objects that are ever active
    active_objs = {a for _, a, _, _ in steps_tl if a}

    # grasp-confirmed time per active object (already computed as grasped_at)
    grasped_at = {}
    for e in (ev["scene"] + ev["motion"] + ev["gripper"]):
        p = e.get("payload", {})
        if e.get("event_type") in ("phase","mark_phase") and str(p.get("phase","")).lower()=="grasped":
            sid=str(p.get("step_id","")); o=p.get("object_id") or sid.replace("repick_","").replace("pick_","")
            if o: grasped_at.setdefault(o, ts(e["wall_time_iso"]))

    per_step = []
    pos_ok = pos_exp = 0
    grip_ok = grip_exp = 0
    grip_late = []
    def _arm_offset(intervals, gt):
        """seconds between grasp-confirm and the nearest arm of this monitor (neg = armed first)."""
        best = None
        for (t0, _t1) in intervals:
            d = (t0 - gt).total_seconds()
            if best is None or abs(d) < abs(best): best = d
        return round(best, 2) if best is not None else None

    for sid, active, w0, w1 in steps_tl:
        expected_here = []   # list of {object, monitor}
        covered_here  = []
        missed_here   = []
        late_here     = []
        offsets_here  = {}   # monitor family -> arm offset vs grasp-confirm (s)
        gates_here    = {}   # monitor family -> arm offset vs gate release (s), None if ungated

        # STATIONARITY: every object except the active one
        for o in sorted(all_objs):
            if o == active: continue
            expected_here.append({"object": o, "monitor": "stationarity"})
            pos_exp += 1
            if is_live(o, "position_static", w0, w1, GRACE):
                pos_ok += 1; covered_here.append({"object": o, "monitor": "stationarity"})
            else:
                missed_here.append({"object": o, "monitor": "stationarity"})

        # EFFORT + SLIP on the active object, only on its grasp step, grasp-anchored.
        # Deadline = the grasped-phase gate release if the run uses that gate (the executor is
        # blocked until then, so arming inside the pause misses no exposure); otherwise the
        # grasp-confirm event itself, since the executor was free to move straight after it.
        if active and (str(sid).startswith("pick") or str(sid).startswith("repick")):
            gt = grasped_at.get(active)
            if gt is not None:
                gate = ready_grasp.get(sid)
                eff_dl  = gate if gate is not None else gt
                slip_dl = gate if gate is not None else gt + GRACE
                gated_here = gate is not None
                # effort
                expected_here.append({"object": active, "monitor": "effort"})
                grip_exp += 1
                eff = live.get(active, {}).get("clamp_effort", [])
                offsets_here["effort"] = _arm_offset(eff, gt)
                gates_here["effort"] = _arm_offset(eff, eff_dl) if gated_here else None
                if any(t0 <= eff_dl and t1 >= eff_dl for (t0, t1) in eff):
                    grip_ok += 1; covered_here.append({"object": active, "monitor": "effort"})
                elif eff:
                    late_here.append({"object": active, "monitor": "effort",
                                      "note": "armed after the executor was released to move"})
                    grip_late.append({"step": sid, "object": active, "monitor": "effort"})
                else:
                    missed_here.append({"object": active, "monitor": "effort"})
                # slip
                expected_here.append({"object": active, "monitor": "slip"})
                grip_exp += 1
                slip = live.get(active, {}).get("grip_relative_pose", [])
                offsets_here["slip"] = _arm_offset(slip, gt)
                gates_here["slip"] = _arm_offset(slip, slip_dl) if gated_here else None
                if any(t0 <= slip_dl and t1 >= slip_dl for (t0, t1) in slip):
                    grip_ok += 1; covered_here.append({"object": active, "monitor": "slip"})
                else:
                    missed_here.append({"object": active, "monitor": "slip"})

        per_step.append({
            "step": sid,
            "active": active,
            "expected": len(expected_here),
            "covered": len(covered_here),
            "missed": missed_here,        # all types, this step
            "late": late_here,            # effort armed too late, if any
            "by_monitor": {
                fam: {
                    "expected": sum(1 for m in expected_here if m["monitor"] == fam),
                    "covered":  sum(1 for m in covered_here  if m["monitor"] == fam),
                    "missed":  [m["object"] for m in missed_here if m["monitor"] == fam],
                    "late":    [m["object"] for m in late_here   if m["monitor"] == fam],
                    "arm_offset_s": offsets_here.get(fam),
                    "gate_margin_s": gates_here.get(fam),
                }
                for fam in ("stationarity", "slip", "effort")
            },
        })

    row["per_step_coverage"] = per_step
    row["stationarity_coverage"] = f"{pos_ok}/{pos_exp}" if pos_exp else "n/a"
    row["grip_coverage"] = f"{grip_ok}/{grip_exp}" if grip_exp else "n/a"
    row["effort_late"] = grip_late
    total_ok, total_exp = pos_ok + grip_ok, pos_exp + grip_exp
    row["arming_coverage"] = f"{total_ok}/{total_exp}" if total_exp else "n/a"

    prov_total = prov_ok = 0; prov_fail = []
    for a in arms:
        obj = _obj(a); fam = FAMILY.get(a.get("monitor_type"), {}); src = a.get("threshold_source")
        if obj in FIXTURES or src == "object_unresolved" or not fam.get("reg_field"): continue
        prov_total += 1
        if a.get("threshold_verified") is True: prov_ok += 1
        else: prov_fail.append({"object": obj, "type": a.get("monitor_type"), "source": src,
                                "registry_value": a.get("threshold_registry_value")})
    row["arming_provenance"] = f"{prov_ok}/{prov_total}" if prov_total else "n/a"
    row["arming_provenance_fail"] = prov_fail

    # ---- INDEPENDENCE ------------------------------------------------------
    exec_status = ended.get("status") if ended else "no_terminal_event"
    row["executor_terminal_status"] = exec_status
    row["watchdog_recorded_exec_status"] = final.get("executor_terminal_status") if final else None
    row["independence_gap"] = (exec_status == "completed" and row["watchdog_final_verdict"] == "non_conformant")
    if step_v:
        row["step_claims"] = [{"step": sv.get("step_id"), "watchdog": sv.get("verdict"),
                               "executor_claimed": sv.get("claimed_outcome")} for sv in step_v]
    return row


# ---------------------------------------------------------------------------
# CSV output — append-only, one row per run. The CSV is the human-editable
# SOURCE OF TRUTH: after scoring, you may correct any cell by hand (you watched
# the run). Audit columns (scorer_raw_*) preserve what the scorer computed, so
# a hand-correction is always visible next to the machine value. The aggregator
# reads the corrected CSV and never recomputes.
# ---------------------------------------------------------------------------
import csv as _csv

# Column order for the per-class results.csv. Editable columns are plain; audit
# columns (scorer_raw_*) mirror the machine value and should NOT be edited.
CSV_COLUMNS = [
    "run_id", "task", "scenario", "class",
    # verdict / cadence
    "watchdog_final_verdict", "expected_final_verdict", "verdict_correct",
    "watchdog_recovered", "expected_recovered_ok", "cadence_compliant",
    # detection (editable) + audit mirrors
    "n_real_violations", "n_TP", "n_FP", "n_FN", "recall", "precision",
    "TP_list", "FP_list", "FN_list",
    "scorer_raw_n_TP", "scorer_raw_n_FP", "scorer_raw_n_FN",
    "scorer_raw_TP_list", "scorer_raw_FP_list", "scorer_raw_FN_list",
    # arming
    "arming_coverage", "stationarity_coverage", "grip_coverage",
    "arming_provenance", "provenance_fail_objs",
    # monitor false alarms (specificity)
    "n_fires", "authored_probe_fires", "monitor_false_alarms",
    # latency / independence
    "time_to_verdict_s", "independence_gap",
    "executor_terminal_status",
    # flags
    "label_consistency_warning", "label_used",
    # human-owned
    "exclude", "manual_note",
]

def _joinpairs(lst):
    return ";".join(lst) if lst else ""

def flatten_row(row):
    det = row.get("detection", {})
    tp, fp, fn = det.get("true_positives", []), det.get("false_positives", []), det.get("false_negatives", [])
    warn = row.get("label_consistency_warning")
    return {
        "run_id": row.get("run_id"), "task": row.get("task"),
        "scenario": row.get("scenario"), "class": row.get("class"),
        "watchdog_final_verdict": row.get("watchdog_final_verdict"),
        "expected_final_verdict": row.get("expected_final_verdict"),
        "verdict_correct": row.get("verdict_correct"),
        "watchdog_recovered": row.get("watchdog_recovered"),
        "expected_recovered_ok": row.get("recovered_correct"),
        "cadence_compliant": row.get("cadence_compliant"),
        "n_real_violations": det.get("n_real_violations"),
        "n_TP": len(tp), "n_FP": len(fp), "n_FN": len(fn),
        "recall": row.get("detection_recall"), "precision": row.get("detection_precision"),
        "TP_list": _joinpairs(tp), "FP_list": _joinpairs(fp), "FN_list": _joinpairs(fn),
        "scorer_raw_n_TP": len(tp), "scorer_raw_n_FP": len(fp), "scorer_raw_n_FN": len(fn),
        "scorer_raw_TP_list": _joinpairs(tp), "scorer_raw_FP_list": _joinpairs(fp),
        "scorer_raw_FN_list": _joinpairs(fn),
        "arming_coverage": row.get("arming_coverage"),
        "stationarity_coverage": row.get("stationarity_coverage"),
        "grip_coverage": row.get("grip_coverage"),
        "arming_provenance": row.get("arming_provenance"),
        "provenance_fail_objs": ";".join(f.get("object","") for f in (row.get("arming_provenance_fail") or [])),
        "n_fires": row.get("n_fires"),
        "authored_probe_fires": row.get("authored_probe_fires"),
        "monitor_false_alarms": row.get("monitor_false_alarms"),
        "time_to_verdict_s": row.get("time_to_verdict_s"),
        "independence_gap": row.get("independence_gap"),
        "executor_terminal_status": row.get("executor_terminal_status"),
        "label_consistency_warning": " | ".join(warn) if warn else "",
        "label_used": row.get("_label_used"),
        "exclude": "",         # human-owned: set to 1 to drop from aggregation
        "manual_note": "",     # human-owned: free text
    }

def _read_existing(csv_path):
    """Read existing rows, tolerating the column-aligned (padded) format we write."""
    if not Path(csv_path).exists(): return []
    with open(csv_path, newline="") as fh:
        raw = [r for r in _csv.reader(fh) if r]
    if not raw: return []
    header = [c.strip() for c in raw[0]]
    bad = [(i, len(r)) for i, r in enumerate(raw[1:], start=2) if len(r) != len(header)]
    if bad:
        detail = "; ".join(f"line {ln} has {n} fields, header has {len(header)}" for ln, n in bad)
        sys.exit(f"ERROR: {csv_path} is misaligned ({detail}).\n"
                 f"       A hand-edit probably deleted or added a comma — every column after the\n"
                 f"       damaged cell is shifted. Fix that line (keep empty cells as bare commas)\n"
                 f"       before re-scoring, otherwise this run would be appended to a broken table.")
    return [{k: (v or "").strip() for k, v in zip(header, r)} for r in raw[1:]]

def _write_aligned(csv_path, rows):
    """Write a column-ALIGNED csv: still comma-separated & parseable, but each field padded to
    its column's max width so it reads like a table in a monospace editor (VSCode). The aggregator
    (and _read_existing) strip the padding on read, so it round-trips cleanly."""
    # compute width per column = max(header, all values)
    widths = {}
    for c in CSV_COLUMNS:
        w = len(c)
        for r in rows:
            w = max(w, len(str(r.get(c, ""))))
        widths[c] = w
    def fmt(vals):
        # pad each cell; last column not padded (avoids trailing spaces on the widest edits)
        cells = []
        for i, c in enumerate(CSV_COLUMNS):
            v = str(vals.get(c, ""))
            # quote any value containing a comma (CSV correctness) BEFORE padding
            if "," in v or '"' in v:
                v = '"' + v.replace('"', '""') + '"'
            cells.append(v.ljust(widths[c]) if i < len(CSV_COLUMNS)-1 else v)
        return ", ".join(cells).rstrip()
    header = {c: c for c in CSV_COLUMNS}
    lines = [fmt(header)] + [fmt(r) for r in rows]
    Path(csv_path).write_text("\n".join(lines) + "\n")

def append_csv(csv_path, flat):
    csv_path = Path(csv_path)
    rows = _read_existing(csv_path)
    if any(r.get("run_id") == flat["run_id"] for r in rows):
        print(f"WARNING: run_id {flat['run_id']!r} already in {Path(csv_path).name} — appending a "
              f"DUPLICATE row. (Append-only expects one row per run; edit the CSV instead of "
              f"re-scoring, or change run_id.)")
    # keep existing rows as-is (preserving any human edits), append the new one, rewrite aligned
    rows.append({c: flat.get(c, "") for c in CSV_COLUMNS})
    _write_aligned(csv_path, rows)
    return csv_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("label", nargs="?", default=None,
                    help="label YAML; if omitted, the single *.yaml inside session_dir is used")
    ap.add_argument("--out", default=None)
    ap.add_argument("--csv", default=None, help="results.csv path (default: <class folder>/results.csv)")
    a = ap.parse_args()
    label_path = a.label
    if label_path is None:
        yamls = sorted(Path(a.session_dir).glob("*.yaml")) + sorted(Path(a.session_dir).glob("*.yml"))
        if len(yamls) == 1:
            label_path = str(yamls[0])
        elif len(yamls) == 0:
            sys.exit(f"ERROR: no label .yaml found in {a.session_dir} — pass one explicitly.")
        else:
            sys.exit(f"ERROR: multiple .yaml in {a.session_dir}: {[y.name for y in yamls]} — pass one explicitly.")
    label = load_label(label_path)
    row = score_run(a.session_dir, label)
    row["_label_used"] = label_path
    out = json.dumps(row, indent=2)
    # 1) full JSON record in the run folder (backup; CSV is the source of truth)
    (Path(a.session_dir) / "score.json").write_text(out)
    # 2) append flat summary row to the CLASS folder's results.csv (parent of run folder),
    #    unless --csv overrides the location.
    class_dir = Path(a.session_dir).resolve().parent
    csv_path = Path(a.csv) if a.csv else (class_dir / "results.csv")
    flat = flatten_row(row)
    append_csv(csv_path, flat)
    print(f"score.json -> {Path(a.session_dir)/'score.json'}")
    print(f"appended row -> {csv_path}")
    if a.out: Path(a.out).write_text(out); print(f"wrote {a.out}")

    # ---- SCORE-FRESH CONSOLE READOUT ----
    W, X = "\033[0m", "\033[0m"
    OK, BAD, HL = "\033[92m", "\033[91m", "\033[96m"
    def mark(b): return f"{OK}OK{X}" if b else f"{BAD}CHECK{X}"
    print("\n" + "="*70)
    print(f"  RUN {HL}{row.get('run_id')}{X}   class={row.get('class')}  scenario={row.get('scenario')}  task={row.get('task')}")
    print(f"  label: {label_path}")
    print("="*70)

    # verdict / cadence / recovery
    vc = row.get("verdict_correct")
    print(f"  VERDICT   watchdog={row.get('watchdog_final_verdict')}  expected={row.get('expected_final_verdict')}   [{mark(vc)}]")
    rc = row.get("recovered_correct")
    print(f"  RECOVERED watchdog={row.get('watchdog_recovered')}  expected={label.get('expected_recovered')}   [{mark(rc) if rc is not None else '-'}]")
    print(f"  CADENCE   one final verdict = {row.get('cadence_compliant')}   [{mark(row.get('cadence_compliant'))}]")
    print(f"  EXECUTOR  terminal_status={row.get('executor_terminal_status')}   independence_gap={row.get('independence_gap')}")

    # detection: expected vs got
    det = row.get("detection", {})
    real = [f"{v.get('step')}:{v.get('object')}" for v in (label.get('violation_steps') or [])]
    print(f"\n  DETECTION  (per step:object)")
    print(f"    expected real violations : {real if real else '(none — clean run)'}")
    print(f"    TP (caught)              : {det.get('true_positives') or '-'}")
    fn = det.get('false_negatives'); fp = det.get('false_positives')
    print(f"    FN (MISSED)              : {BAD+str(fn)+X if fn else '-'}")
    print(f"    FP (false alarm)         : {BAD+str(fp)+X if fp else '-'}")
    print(f"    recall={row.get('detection_recall')}  precision={row.get('detection_precision')}")
    if row.get('monitor_false_alarms'):
        print(f"    monitor false alarms (fires on a clean run): {row.get('monitor_false_alarms')}")
    if row.get('authored_probe_fires'):
        print(f"    authored probe/debug fires (excluded from false alarms): "
              f"{row.get('authored_probe_fires')}")

    # arming coverage with the actual misses laid out
    print(f"\n  ARMING")
    print(f"    coverage={row.get('arming_coverage')}  (stationarity {row.get('stationarity_coverage')}, grip {row.get('grip_coverage')})")
    print(f"    provenance={row.get('arming_provenance')}", end="")
    pf = row.get('arming_provenance_fail') or []
    if pf:
        print(f"   {BAD}fail:{X}")
        for f in pf:
            print(f"       {f['object']} / {f.get('type')}: source={f.get('source')} "
                  f"registry={f.get('registry_value')}")
    else: print()
    late = row.get('effort_late') or []

    # per-step arming table: one column per monitor family, so stationarity, relative-pose
    # (slip) and clamp-effort are all visible, not just stationarity.
    per_step = row.get('per_step_coverage', [])
    def pad(txt, w, color=None):
        s = txt.ljust(w)
        return f"{color}{s}{X}" if color else s
    print(f"\n    per-step arming   stat=stationarity on idle objects | "
          f"slip=grip_relative_pose, effort=clamp_effort on the active object")
    SW = max([len('step')] + [len(str(s.get('step') or '')) for s in per_step])
    AW = max([len('active')] + [len(str(s.get('active') or '-')) for s in per_step])
    print(f"      {'step':<{SW}} {'active':<{AW}} {'stat':<9} {'slip':<14} {'effort':<14}")
    for s in per_step:
        bm = s.get('by_monitor') or {}
        st = bm.get('stationarity') or {}
        se, sc = st.get('expected', 0), st.get('covered', 0)
        stat_cell = pad(f"{sc}/{se}" if se else "-", 9, BAD if st.get('missed') else (OK if se else None))
        def grip_cell(fam, w=14):
            d = bm.get(fam) or {}
            if not d.get('expected'): return pad("-", w)
            gm, off = d.get('gate_margin_s'), d.get('arm_offset_s')
            # gated runs: show margin vs the gate release (negative = armed before release)
            tag = (f"{gm:+.1f}s" if gm is not None
                   else (f"{off:+.1f}s" if off is not None else "no-arm"))
            if d.get('missed'): return pad(f"MISS {tag}", w, BAD)
            if d.get('late'):   return pad(f"LATE {tag}", w, BAD)
            return pad(f"OK {tag}", w, OK)
        print(f"      {s['step']:<{SW}} {(s.get('active') or '-'):<{AW}} "
              f"{stat_cell} {grip_cell('slip')} {grip_cell('effort')}")
    gated = any((((s.get('by_monitor') or {}).get('effort') or {}).get('gate_margin_s') is not None)
                for s in per_step)
    print("      (slip/effort offset = arm time minus " +
          ("grasped-gate release; negative = armed before the executor was released)"
           if gated else "grasp-confirm; negative = armed before grasp)"))

    # misses broken out by family, not just stationarity
    misses = [(s['step'], s['missed']) for s in per_step if s['missed']]
    if misses or late:
        print(f"\n    {BAD}coverage misses by step:{X}")
        for step, ms in misses:
            for fam in ("stationarity", "slip", "effort"):
                objs = [m['object'] for m in ms if m['monitor'] == fam]
                if objs:
                    print(f"       {step:<{SW}} {fam:<13} {', '.join(objs)}")
        for l in late:
            print(f"       {l['step']:<{SW}} {l['monitor']:<13} {l['object']} (armed after grasp-confirm)")
    else:
        print(f"    {OK}no coverage misses{X}")

    # latency
    print(f"\n  LATENCY   time_to_verdict={row.get('time_to_verdict_s')}s")
    if row.get('fire_to_finding_s'):
        print(f"            fire->finding={row.get('fire_to_finding_s')}s")

    # top-line + warnings
    warn = row.get("label_consistency_warning")
    print("\n" + "-"*70)
    allgood = vc and row.get('cadence_compliant') and not warn and not fn and not fp
    if warn:
        print(f"  {BAD}!! LABEL WARNING:{X}")
        for wln in warn: print(f"     - {wln}")
    print(f"  OVERALL: {OK+'looks correct — CSV row appended'+X if allgood else BAD+'REVIEW — check flagged items above, correct CSV if needed'+X}")
    print("="*70)

if __name__ == "__main__":
    sys.exit(main())