#!/usr/bin/env python3
"""
watchdog_scorer.py — score one WatchDog run against a per-run ground-truth label.

FINALIZED for the always-final-verdict cadence: every run emits exactly one final run_verdict,
plus zero or more step-scoped findings. Scores verdict, cadence, detection (TP/FP/TN/FN),
arming (coverage/provenance/type with WINDOW TIMING enforced), latency, and independence.

USAGE:  python3 watchdog_scorer.py <session_dir> <label.yaml> [--out row.json]
        python3 watchdog_scorer.py --summarize results.csv [more.csv ...]

SCENARIOS (this scorer covers S1/S2/S3; S4 has its own scorer, s4_scorer.py):
  S1 = library primitives only; observation only via look() at step boundaries
  S2 = library primitives + authored monitors (arm_custom_monitor)
  S3 = authored monitors only
S2 and S3 may also run continuous watch monitors (purpose="observation").

MONITOR ORIGIN SPLIT (never merged into one accuracy number):
  - LIBRARY monitors keep the family-based scoring below (family coverage,
    window timing, registry provenance).
  - AUTHORED monitors are scored BOOLEAN per injected fault: TP/FP/FN/TN,
    reported in their own columns (authored_*). No family is inferred for them.
  - purpose=="observation" monitors (agent-chosen thresholds that only trigger
    a look, never judge a clause) are EXCLUDED from detection, false-alarm and
    parameter-accuracy metrics; their count is reported separately.
  - Release-time fires of authored monitors on the just-released object get
    their own bucket (authored monitors have no release auto-disarm by design)
    and are never counted as FP.

Scene-observation quality (intruders, ground truth, engagement) is scored by
the SEPARATE observation_scorer.py — its metrics live in their own table.

TERMINAL MODEL (post-cadence-change):
  - Final verdict is ALWAYS present: run_verdict.verdict in {conformant, non_conformant,
    incomplete}. This is the END-STATE assessment.
  - Step findings (event_type 'verdict', verdict != conformant/valid_variation) are the TIMELINE
    of what went wrong and when.
  - recovered = bool(findings) and final verdict conformant  (violation caught, then corrected).
  - The (final_verdict, recovered) PAIR distinguishes: clean (conformant,False),
    recovery (conformant,True), broken (non_conformant,-), aborted (incomplete,-).

DETECTION SCORING — two SEPARATE confusion matrices, never merged or averaged:
  verdict_TP/FP/TN/FN  the RUN-LEVEL verdict call: did the final run_verdict flag a violation
                       (non_conformant) when the label expected one. Positive class =
                       non_conformant; "incomplete" counts as not-flagged on both sides, and
                       exact three-way correctness stays in verdict_correct.
  fault_TP/FP/TN/FN    the STEP-LEVEL pairing: each watchdog finding matched to a real
                       violation per (step, object) as before. fault_TN = steps with neither a
                       real violation nor a finding.
  The two can legitimately disagree on one run (correct final verdict, missed step-level
  pairing) — that disagreement is signal, so they are reported side by side.

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
from datetime import datetime, timezone
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

def _origin(p):   return p.get("origin") or "library"
def _purpose(p):  return p.get("purpose") or "clause"

def _median(xs):
    xs = sorted(xs)
    if not xs: return None
    n = len(xs)
    return round(xs[n//2] if n % 2 else (xs[n//2 - 1] + xs[n//2]) / 2, 2)

def _p95(xs):
    xs = sorted(xs)
    if not xs: return None
    # nearest-rank p95 (ceil), correct for the small-n lists runs produce
    import math as _m
    return round(xs[max(0, _m.ceil(0.95 * len(xs)) - 1)], 2)


def score_authored_faults(real_pairs, authored_clause_fires, step_window,
                          release_windows, violation_occurred,
                          grace_s=3.0):
    """Boolean per injected fault for AUTHORED monitors: TP/FP/FN (+TN).

    Shared by watchdog_scorer (S2/S3) and s4_scorer (where ALL monitors are
    authored). No family is inferred for authored monitors — a fault is either
    caught by an authored fire or it is not.

    real_pairs            {(step_id, object)} ground-truth violations (label).
    authored_clause_fires payload dicts (wall_time_iso present), ALREADY
                          excluding probe fires and purpose=="observation".
    step_window           callable step_id -> (t0, t1) datetimes (t1 None = run end).
    release_windows       [(object_id, t0, t1)] spans where an authored fire on
                          that object is a RELEASE ARTIFACT (authored monitors
                          have no release auto-disarm by design) — own bucket,
                          never FP.
    violation_occurred    label bool, for the TN cell.

    Returns {"tp": [...], "fp": [...], "fn": [...], "release_fires": [...],
             "tn": bool|None} — tn is only defined on clean runs.
    """
    from datetime import timedelta
    grace = timedelta(seconds=float(grace_s))

    def _fire_row(f):
        return {"object": f.get("object_id"), "monitor_id": f.get("monitor_id"),
                "value": f.get("triggering_value"), "threshold": f.get("threshold"),
                "time": f.get("wall_time_iso")}

    matched_pairs, fp, release_fires = set(), [], []
    for f in authored_clause_fires:
        ft, obj = ts(f["wall_time_iso"]), f.get("object_id")
        if any(o == obj and t0 <= ft <= t1 for (o, t0, t1) in release_windows):
            release_fires.append(_fire_row(f))
            continue
        hit = False
        for (s, o) in real_pairs:
            if o != obj and obj is not None:
                continue
            t0, t1 = step_window(s)
            if t0 is None:
                continue
            if ft >= t0 - grace and (t1 is None or ft <= t1 + grace):
                matched_pairs.add((s, o)); hit = True
        if not hit:
            fp.append(_fire_row(f))
    fn = real_pairs - matched_pairs
    tn = None
    if not violation_occurred:
        tn = (len(fp) == 0)
    def pretty(pairs):
        return [f"{s}:{o}" for (s, o) in sorted(pairs, key=lambda x: (x[0] or '', x[1] or ''))]
    return {"tp": pretty(matched_pairs), "fp": fp, "fn": pretty(fn),
            "release_fires": release_fires, "tn": tn}

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
def step_active_object(payload, known_objects):
    """The step's active object: begin_step object_ids[0]; when the executor
    omitted object_ids, the longest known object id named in the step_id
    (pick_u_orange -> u_orange, pick_u_brown_retry -> u_brown) -- the same
    fallback the watchdog's contract uses (process order when object_ids is
    missing). None when neither resolves."""
    ids = payload.get("object_ids") or []
    if ids:
        return ids[0]
    sid = str(payload.get("step_id") or "")
    hits = [o for o in known_objects if o and o in sid]
    return max(hits, key=len) if hits else None


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
    scn = str(label.get("scenario") or "").upper()

    # ---- monitor ORIGIN / PURPOSE partition --------------------------------
    # Library and authored monitors are scored in SEPARATE columns, never one
    # merged accuracy number. purpose=="observation" monitors (agent-chosen
    # look-trigger thresholds) are excluded from detection/false-alarm/parameter
    # metrics entirely — they are a legitimate observation strategy, scored by
    # observation_scorer.py.
    lib_arms         = [a for a in arms if _origin(a) == "library"]
    auth_arms        = [a for a in arms if _origin(a) == "authored"]
    obs_arms         = [a for a in auth_arms if _purpose(a) == "observation"]
    auth_clause_arms = [a for a in auth_arms if _purpose(a) != "observation"]
    obs_fires        = [f for f in fires if _purpose(f) == "observation"]
    clause_fires     = [f for f in fires if _purpose(f) != "observation"]
    row["n_library_arms"]  = len(lib_arms)
    row["n_authored_arms"] = len(auth_arms)
    row["n_authored_obs_monitors"] = len({a.get("monitor_id") for a in obs_arms})
    row["n_observation_fires"] = len(obs_fires)

    # scenario consistency: what was armed must match what the scenario allows
    scn_warns = []
    if scn == "S1" and auth_arms:
        scn_warns.append(f"S1 run has {len(auth_arms)} authored monitor arms — "
                         "S1 is library-primitives only; check the scenario label / mcp config.")
    if scn == "S3" and lib_arms:
        scn_warns.append(f"S3 run has {len(lib_arms)} library monitor arms — "
                         "S3 is authored-only; check the scenario label / mcp config.")
    row["scenario_consistency_warning"] = scn_warns or None

    # ---- final verdict (ALWAYS present now) --------------------------------
    final = run_v[-1] if run_v else None
    row["watchdog_final_verdict"] = final["verdict"] if final else None
    row["watchdog_recovered"]     = final.get("recovered") if final else None
    row["findings_during_run"]    = final.get("findings_during_run") if final else None
    row["n_run_verdicts"]         = len(run_v)

    exp_fv = label.get("expected_final_verdict", label.get("expected_verdict"))
    row["expected_final_verdict"] = exp_fv
    row["verdict_correct"] = (row["watchdog_final_verdict"] == exp_fv)

    # ---- VERDICT confusion cells (run-level; SEPARATE from the fault_* step
    # pairing below — the two can legitimately disagree on one run and are
    # never merged). Positive = non_conformant; "incomplete" counts as
    # not-flagged on both sides (three-way exactness stays in verdict_correct).
    exp_pos = (exp_fv == "non_conformant")
    got_pos = (row["watchdog_final_verdict"] == "non_conformant")
    row["verdict_TP"] = int(exp_pos and got_pos)
    row["verdict_FN"] = int(exp_pos and not got_pos)
    row["verdict_FP"] = int(not exp_pos and got_pos)
    row["verdict_TN"] = int(not exp_pos and not got_pos)

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
    # observation-purpose fires never attribute a conformance finding — they are
    # look triggers, not clause judgements.
    fires_ev = [e for e in wd_events(ev, "monitor_fire")
                if _purpose(_pl(e)) != "observation"]
    finding_steps = {sv.get("step_id") for sv in step_v if sv.get("verdict") in FINDING_VERDICTS}
    known_objects = {m.get("object") for m in label.get("expected_monitors", []) or []}
    step_active = {b["payload"].get("step_id"): step_active_object(b["payload"], known_objects)
                   for b in begins}
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
        # also the verdict's own clause object, if any. A finding verdict with
        # no clause object and NO fire of its own (none since the step began
        # or since the previous finding verdict at this step) judged by direct
        # measurement (e.g. query_snapshot at end_step) — attribute it to the
        # step's ACTIVE object, so another object's fire in the same step
        # cannot absorb it.
        prev_t = t0
        for sv in sorted((sv for sv in step_v if sv.get("step_id") == sid
                          and sv.get("verdict") in FINDING_VERDICTS),
                         key=lambda sv: sv.get("wall_time_iso") or ""):
            co = (sv.get("clause") or {}).get("object_id")
            if co:
                objs.add(co)
            vt = ts(sv["wall_time_iso"]) if sv.get("wall_time_iso") else None
            if not co and vt is not None and step_active.get(sid) and not any(
                    (prev_t is None or ts(e["wall_time_iso"]) > prev_t)
                    and ts(e["wall_time_iso"]) <= vt
                    and (t0 is None or ts(e["wall_time_iso"]) >= t0)
                    for e in fires_ev):
                objs.add(step_active[sid])
            if vt is not None:
                prev_t = vt
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
    # fault_TN: steps with neither a real violation nor a finding (step-level cell;
    # the (step, object) pairing has no natural negative universe per object).
    all_steps = {s for (_, s) in bs_win if s}
    tn_steps = all_steps - {s for (s, _) in real_pairs} - {s for (s, _) in finding_pairs}
    row["detection"] = {
        "true_positives":  pretty(tp),
        "false_negatives": pretty(fn),
        "false_positives": pretty(fp),
        "clean_steps_no_finding": sorted(tn_steps),
        "n_real_violations": len(real_pairs),
        "n_findings": len(finding_pairs),
    }
    row["fault_TN"] = len(tn_steps)
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
    # false-alarm universe = CLAUSE fires only: observation-purpose fires are a
    # look-trigger strategy, not an alarm about conformance.
    real_fires  = [f for f in clause_fires if not _is_probe(f)]
    probe_fires = [f for f in clause_fires if _is_probe(f)]
    n_fires = len(fires)
    row["n_fires"] = n_fires                     # total, probes + observation included
    row["authored_probe_fires"] = len(probe_fires)

    # RELEASE WINDOWS: authored monitors have no release auto-disarm by design,
    # so an authored fire on the just-released object during/right after the
    # release is an artifact of that design, bucketed on its own — never FP,
    # never a false alarm. Window: release_skill start -1s .. end +5s.
    from datetime import timedelta as _td
    release_windows = []
    rel_open = {}   # object_id -> start time
    for e in ev["gripper"]:
        p = e.get("payload", {})
        if p.get("skill_name") != "release_skill":
            continue
        obj = (p.get("params") or {}).get("object_id")
        t = ts(e["wall_time_iso"])
        if e.get("event_type") == "skill_start":
            rel_open[obj] = t
        elif e.get("event_type") == "skill_end":
            t0 = rel_open.pop(obj, t)
            release_windows.append((obj, t0 - _td(seconds=1), t + _td(seconds=5)))
    for obj, t0 in rel_open.items():   # start with no end: window to +10s
        release_windows.append((obj, t0 - _td(seconds=1), t0 + _td(seconds=10)))

    def _in_release_window(f):
        ft, obj = ts(f["wall_time_iso"]), f.get("object_id")
        return any(o == obj and t0 <= ft <= t1 for (o, t0, t1) in release_windows)

    if not label.get("violation_occurred"):
        fa = [f for f in real_fires
              if not (_origin(f) == "authored" and _in_release_window(f))]
        row["monitor_false_alarms"] = len(fa)    # a REAL clause fire on a clean run
        row["monitor_false_alarm_detail"] = [
            {"object": f.get("object_id"), "type": f.get("monitor_type"),
             "origin": _origin(f),
             "value": f.get("triggering_value"), "threshold": f.get("threshold")} for f in fa]
    else:
        row["monitor_false_alarms"] = None      # fires may be true detections; see detection block

    # ---- AUTHORED monitors: boolean per injected fault (TP/FP/FN/TN) --------
    # Own columns, never merged with the library family scoring. S1 should have
    # no authored monitors at all — columns stay n/a there.
    if scn in ("S2", "S3") or auth_arms:
        authored_clause_fires = [f for f in real_fires if _origin(f) == "authored"]
        auth = score_authored_faults(
            real_pairs, authored_clause_fires, step_window, release_windows,
            bool(label.get("violation_occurred")),
            grace_s=float(label.get("boundary_grace_s", 3.0)))
        row["authored_detection"] = auth
        row["authored_TP"] = len(auth["tp"]); row["authored_FP"] = len(auth["fp"])
        row["authored_FN"] = len(auth["fn"]); row["authored_TN"] = auth["tn"]
        row["authored_release_false_fires"] = len(auth["release_fires"])
        row["authored_release_fire_detail"] = auth["release_fires"] or None
    else:
        row["authored_detection"] = None
        row["authored_TP"] = row["authored_FP"] = row["authored_FN"] = None
        row["authored_TN"] = None
        row["authored_release_false_fires"] = None

    # ---- TIME TO VERDICT ---------------------------------------------------
    ttv = None
    if final and ended:
        ttv = (ts(final["wall_time_iso"]) - ts(exec_events(ev, "run_ended")[0]["wall_time_iso"])).total_seconds()
    row["time_to_verdict_s"] = round(ttv, 2) if ttv is not None else None

    # ---- FIRE -> VERDICT latency (per fire, HEURISTIC pairing) --------------
    # Fire time = the payload's trigger_wall_ns: the engine stamps it with
    # wall_ns() at trigger, so it is wall clock and comparable to the verdict
    # envelope's wall_time_iso (never the fire's own envelope time, which is
    # when get_monitor_fires DRAINED it, nor timestamp_ns, which is
    # per-process monotonic). No verdict record identifies which fire it
    # adjudicates, so pairing is heuristic — FIRST verdict at/after the fire
    # whose step_id matches the step the fire landed in, else the final
    # run_verdict if its clause_results cover that step. The column names
    # carry "next_verdict" so the heuristic stays visible. Probe and
    # observation-purpose fires are excluded; a fire without trigger_wall_ns
    # (pre-schema session) is skipped and counted apart, never measured by
    # drain time.
    verdicts_t = sorted((ts(sv["wall_time_iso"]), sv.get("step_id")) for sv in step_v)
    final_t = ts(final["wall_time_iso"]) if final else None
    final_steps = {str(c.get("step_id")) for c in (final.get("clause_results") or [])} \
        if final else set()

    def _fire_step(ft):
        for i, (t, s) in enumerate(bs_win):
            t0 = ts(t); t1 = ts(bs_win[i+1][0]) if i+1 < len(bs_win) else None
            if ft >= t0 and (t1 is None or ft < t1):
                return s
        return None

    gaps, n_adj, n_never, n_no_stamp = [], 0, 0, 0
    for f in real_fires:
        if f.get("trigger_wall_ns") is None:
            n_no_stamp += 1
            continue
        ft = datetime.fromtimestamp(int(f["trigger_wall_ns"]) / 1e9, tz=timezone.utc)
        fstep = _fire_step(ft)
        vt = next((t for (t, s) in verdicts_t if t >= ft and s == fstep), None)
        if vt is None and final_t is not None and final_t >= ft \
                and str(fstep) in final_steps:
            vt = final_t
        if vt is not None:
            n_adj += 1
            gaps.append(round((vt - ft).total_seconds(), 2))
        else:
            n_never += 1
    row["fire_to_next_verdict_s"] = gaps or None
    row["fire_to_next_verdict_median_s"] = _median(gaps)
    row["fire_to_next_verdict_p95_s"] = _p95(gaps)
    row["n_fires_adjudicated"] = n_adj
    row["n_fires_never_adjudicated"] = n_never
    row["n_fires_missing_trigger_stamp"] = n_no_stamp

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
        p = _pl(e); events_by_id[p.get("monitor_id")].append(
            (ts(p["wall_time_iso"]), "disarm", None, p.get("reason")))
    # object -> family_kind -> list of (start,end) live intervals
    live = defaultdict(lambda: defaultdict(list))
    # (object, family_kind, end) of intervals that ended because the monitor
    # FIRED (one-shot auto-disarm): the monitor did its job, so is_live counts
    # it as covering the rest of the step window it fired in.
    fired_ends = set()
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
            else:  # disarm (mtype slot carries the disarm reason)
                if open_arm is not None:
                    pt0, po, pm = open_arm; fam = FAMILY.get(pm)
                    if fam and po:
                        live[po][fam["kind"]].append((pt0, t))
                        if mtype == "fired":
                            fired_ends.add((po, fam["kind"], t))
                    open_arm = None
        if open_arm is not None:  # never disarmed -> live to run end
            pt0, po, pm = open_arm; fam = FAMILY.get(pm)
            if fam and po: live[po][fam["kind"]].append((pt0, end_all))
    def is_live(o, kind, w0, w1, grace):
        """monitor of `kind` on `o` live for the BULK of [w0,w1] (grace at each boundary)."""
        need0, need1 = w0 + grace, w1 - grace
        if need1 <= need0: need0, need1 = w0, w1   # step shorter than 2*grace: require any overlap
        for (t0, t1) in live.get(o, {}).get(kind, []):
            if (o, kind, t1) in fired_ends and w0 <= t1 <= w1:
                t1 = w1   # fired inside this window: covered to its end
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
                   step_active_object(b["payload"], known_objects) )
                 for b in begins], key=lambda x: x[0])
    steps_tl = []
    _ends = exec_events(ev, "run_ended")
    run_end_t = ts(_ends[0]["wall_time_iso"]) if _ends else None
    # next step's (begin_step time, active object) per step — see the
    # stationarity loop: the next target's stationarity is only owed until
    # the executor blocks at the next gate, not until that gate's release.
    next_target = {}
    for i, (t, sid, active) in enumerate(bs):
        # window start = when the executor was RELEASED to act on this step
        w0 = ready_step.get(sid, ts(t))
        # window end = when the NEXT step was released (or run end)
        if i+1 < len(bs):
            nxt_sid = bs[i+1][1]
            w1 = ready_step.get(nxt_sid, ts(bs[i+1][0]))
            next_target[sid] = (ts(bs[i+1][0]), bs[i+1][2])
        else:
            # last step: ends when the executor ended the run, not at the
            # watchdog's last event — disarming after run_ended (wrap-up
            # before the run verdict) is cleanup, not a coverage gap.
            w1 = min(end_all, run_end_t) if run_end_t else end_all
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

        # STATIONARITY: every object except the active one. For the NEXT
        # step's active object the window ends at the next begin_step, not the
        # next release: during the gate pause the executor is blocked and the
        # watchdog must disarm that object's stationarity before releasing the
        # grasp, so a disarm inside the pause is correct preparation, not a
        # coverage miss (decided 2026-10-07, before the first valid S1 run).
        nxt_begin, nxt_active = next_target.get(sid, (None, None))
        for o in sorted(all_objs):
            if o == active: continue
            expected_here.append({"object": o, "monitor": "stationarity"})
            pos_exp += 1
            w1_o = nxt_begin if (o == nxt_active and nxt_begin is not None
                                 and w0 < nxt_begin < w1) else w1
            if is_live(o, "position_static", w0, w1_o, GRACE):
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

    # AUTHORED parameter accuracy: a clause-judging authored monitor's threshold
    # must trace to the registry like a library one (threshold_verified at arm
    # time). purpose=="observation" thresholds are AGENT-CHOSEN by design — they
    # only trigger a look, never judge a clause — so they are reported in their
    # own count and NEVER counted as parameter errors.
    ap_total = ap_ok = 0; ap_fail = []
    for a in auth_clause_arms:
        if a.get("threshold") is None:    # no scalar threshold: nothing to verify
            continue
        obj = _obj(a)
        if obj in FIXTURES:
            continue
        ap_total += 1
        if a.get("threshold_verified") is True:
            ap_ok += 1
        else:
            ap_fail.append({"object": obj, "monitor_id": a.get("monitor_id"),
                            "threshold": a.get("threshold"),
                            "source": a.get("threshold_source"),
                            "registry_value": a.get("threshold_registry_value")})
    row["authored_param_accuracy"] = f"{ap_ok}/{ap_total}" if ap_total else "n/a"
    row["authored_param_fail"] = ap_fail
    row["agent_chosen_obs_thresholds"] = len(obs_arms)

    # S3 is authored-only: the library family coverage grid does not apply, and
    # leaving it populated would read as wall-to-wall misses. Blanked AFTER
    # computation so S1/S2 behavior is untouched.
    if scn == "S3":
        row["per_step_coverage"] = []
        row["stationarity_coverage"] = "n/a"
        row["grip_coverage"] = "n/a"
        row["arming_coverage"] = "n/a"
        row["effort_late"] = []

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
    # run-level verdict confusion (positive = non_conformant; incomplete =
    # not-flagged; exact three-way correctness stays in verdict_correct)
    "verdict_TP", "verdict_FP", "verdict_TN", "verdict_FN",
    # step-level fault pairing (editable) + audit mirrors — SEPARATE from the
    # verdict_* group above; the two can disagree on one run by design
    "n_real_violations", "fault_TP", "fault_FP", "fault_TN", "fault_FN",
    "fault_recall", "fault_precision",
    "fault_TP_list", "fault_FP_list", "fault_FN_list",
    "scorer_raw_fault_TP", "scorer_raw_fault_FP", "scorer_raw_fault_FN",
    "scorer_raw_fault_TP_list", "scorer_raw_fault_FP_list", "scorer_raw_fault_FN_list",
    # authored monitors — boolean per injected fault, own columns (never merged
    # with the library family columns)
    "n_authored_arms", "authored_TP", "authored_FP", "authored_FN", "authored_TN",
    "authored_release_false_fires", "authored_param_accuracy",
    # observation-purpose monitors (agent-chosen thresholds; excluded elsewhere)
    "n_authored_obs_monitors", "agent_chosen_obs_thresholds", "n_observation_fires",
    # arming (LIBRARY lane)
    "arming_coverage", "stationarity_coverage", "grip_coverage",
    "arming_provenance", "provenance_fail_objs",
    # monitor false alarms (specificity; clause fires only)
    "n_fires", "authored_probe_fires", "monitor_false_alarms",
    # latency / independence. fire_to_NEXT_verdict: pairing is heuristic
    # (first same-step verdict after the fire's trigger_wall_ns) — no verdict
    # record names the fire it adjudicates.
    "time_to_verdict_s", "fire_to_next_verdict_median_s", "fire_to_next_verdict_p95_s",
    "n_fires_adjudicated", "n_fires_never_adjudicated",
    "independence_gap",
    "executor_terminal_status",
    # flags
    "label_consistency_warning", "scenario_consistency_warning", "label_used",
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
        "verdict_TP": row.get("verdict_TP"), "verdict_FP": row.get("verdict_FP"),
        "verdict_TN": row.get("verdict_TN"), "verdict_FN": row.get("verdict_FN"),
        "n_real_violations": det.get("n_real_violations"),
        "fault_TP": len(tp), "fault_FP": len(fp), "fault_TN": row.get("fault_TN"),
        "fault_FN": len(fn),
        "fault_recall": row.get("detection_recall"),
        "fault_precision": row.get("detection_precision"),
        "fault_TP_list": _joinpairs(tp), "fault_FP_list": _joinpairs(fp),
        "fault_FN_list": _joinpairs(fn),
        "scorer_raw_fault_TP": len(tp), "scorer_raw_fault_FP": len(fp),
        "scorer_raw_fault_FN": len(fn),
        "scorer_raw_fault_TP_list": _joinpairs(tp), "scorer_raw_fault_FP_list": _joinpairs(fp),
        "scorer_raw_fault_FN_list": _joinpairs(fn),
        "n_authored_arms": row.get("n_authored_arms"),
        "authored_TP": row.get("authored_TP"), "authored_FP": row.get("authored_FP"),
        "authored_FN": row.get("authored_FN"), "authored_TN": row.get("authored_TN"),
        "authored_release_false_fires": row.get("authored_release_false_fires"),
        "authored_param_accuracy": row.get("authored_param_accuracy"),
        "n_authored_obs_monitors": row.get("n_authored_obs_monitors"),
        "agent_chosen_obs_thresholds": row.get("agent_chosen_obs_thresholds"),
        "n_observation_fires": row.get("n_observation_fires"),
        "arming_coverage": row.get("arming_coverage"),
        "stationarity_coverage": row.get("stationarity_coverage"),
        "grip_coverage": row.get("grip_coverage"),
        "arming_provenance": row.get("arming_provenance"),
        "provenance_fail_objs": ";".join(f.get("object","") for f in (row.get("arming_provenance_fail") or [])),
        "n_fires": row.get("n_fires"),
        "authored_probe_fires": row.get("authored_probe_fires"),
        "monitor_false_alarms": row.get("monitor_false_alarms"),
        "time_to_verdict_s": row.get("time_to_verdict_s"),
        "fire_to_next_verdict_median_s": row.get("fire_to_next_verdict_median_s"),
        "fire_to_next_verdict_p95_s": row.get("fire_to_next_verdict_p95_s"),
        "n_fires_adjudicated": row.get("n_fires_adjudicated"),
        "n_fires_never_adjudicated": row.get("n_fires_never_adjudicated"),
        "independence_gap": row.get("independence_gap"),
        "executor_terminal_status": row.get("executor_terminal_status"),
        "label_consistency_warning": " | ".join(warn) if warn else "",
        "scenario_consistency_warning": " | ".join(row.get("scenario_consistency_warning") or []),
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

def _write_aligned(csv_path, rows, columns=None):
    """Write a column-ALIGNED csv: still comma-separated & parseable, but each field padded to
    its column's max width so it reads like a table in a monospace editor (VSCode). The aggregator
    (and _read_existing) strip the padding on read, so it round-trips cleanly.
    `columns` lets observation_scorer / s4_scorer reuse this with their own tables."""
    cols = columns or CSV_COLUMNS
    # compute width per column = max(header, all values)
    widths = {}
    for c in cols:
        w = len(c)
        for r in rows:
            w = max(w, len(str(r.get(c, ""))))
        widths[c] = w
    def fmt(vals):
        # pad each cell; last column not padded (avoids trailing spaces on the widest edits)
        cells = []
        for i, c in enumerate(cols):
            v = str(vals.get(c, ""))
            # quote any value containing a comma (CSV correctness) BEFORE padding
            if "," in v or '"' in v:
                v = '"' + v.replace('"', '""') + '"'
            cells.append(v.ljust(widths[c]) if i < len(cols)-1 else v)
        return ", ".join(cells).rstrip()
    header = {c: c for c in cols}
    lines = [fmt(header)] + [fmt(r) for r in rows]
    Path(csv_path).parent.mkdir(parents=True, exist_ok=True)
    Path(csv_path).write_text("\n".join(lines) + "\n")

def append_csv(csv_path, flat, columns=None):
    cols = columns or CSV_COLUMNS
    csv_path = Path(csv_path)
    rows = _read_existing(csv_path)
    if any(r.get("run_id") == flat["run_id"] for r in rows):
        print(f"WARNING: run_id {flat['run_id']!r} already in {Path(csv_path).name} — appending a "
              f"DUPLICATE row. (Append-only expects one row per run; edit the CSV instead of "
              f"re-scoring, or change run_id.)")
    # keep existing rows as-is (preserving any human edits), append the new one, rewrite aligned
    rows.append({c: flat.get(c, "") for c in cols})
    _write_aligned(csv_path, rows, cols)
    return csv_path


# ---------------------------------------------------------------------------
# Per-scenario summary over one or more results.csv files (after any hand
# corrections — the aggregator reads the CSV and never recomputes).
# ---------------------------------------------------------------------------
def summarize(csv_paths):
    rows = []
    for p in csv_paths:
        rows += _read_existing(p)
    rows = [r for r in rows if str(r.get("exclude", "")).strip() not in ("1", "true", "True", "x")]
    by_scn = defaultdict(list)
    for r in rows:
        by_scn[str(r.get("scenario", "?")).upper()].append(r)

    def _num(r, k):
        v = str(r.get(k, "")).strip()
        try: return float(v)
        except ValueError: return None
    def _nums(rs, *keys):
        # first key present per row wins — lets one summary span this scorer's
        # fault_*/fire_to_next_* CSVs and s4_scorer's older n_TP/fire_to_verdict names
        out = []
        for r in rs:
            for k in keys:
                v = _num(r, k)
                if v is not None:
                    out.append(v); break
        return out
    def _frac(rs, k):
        """average of 'a/b' cells, pooled numerator/denominator."""
        a = b = 0
        for r in rs:
            cell = str(r.get(k, "")).strip()
            if "/" in cell:
                try:
                    x, y = cell.split("/"); a += int(x); b += int(y)
                except ValueError: pass
        return f"{a}/{b}" if b else "n/a"
    def _avg(xs): return round(sum(xs)/len(xs), 3) if xs else None

    print(f"\n{'='*78}\n  PER-SCENARIO SUMMARY   ({len(rows)} runs from {len(csv_paths)} csv)\n{'='*78}")
    for scn in sorted(by_scn):
        rs = by_scn[scn]
        n_vc = sum(1 for r in rs if str(r.get("verdict_correct")) == "True")
        print(f"\n  {scn}  ({len(rs)} runs)")
        print(f"    verdict_correct        : {n_vc}/{len(rs)}")
        print(f"    verdict TP/FP/TN/FN    : {int(sum(_nums(rs,'verdict_TP')))}/"
              f"{int(sum(_nums(rs,'verdict_FP')))}/{int(sum(_nums(rs,'verdict_TN')))}/"
              f"{int(sum(_nums(rs,'verdict_FN')))}   (run-level verdict call)")
        print(f"    fault recall/prec      : avg {_avg(_nums(rs,'fault_recall','recall'))} / "
              f"{_avg(_nums(rs,'fault_precision','precision'))}")
        print(f"    fault TP/FP/TN/FN      : {int(sum(_nums(rs,'fault_TP','n_TP')))}/"
              f"{int(sum(_nums(rs,'fault_FP','n_FP')))}/{int(sum(_nums(rs,'fault_TN')))}/"
              f"{int(sum(_nums(rs,'fault_FN','n_FN')))}   (step-level pairing)")
        if scn in ("S2", "S3", "S4"):
            atp, afp = sum(_nums(rs,'authored_TP')), sum(_nums(rs,'authored_FP'))
            afn = sum(_nums(rs,'authored_FN'))
            tns = [str(r.get('authored_TN')) for r in rs if str(r.get('authored_TN')).strip() not in ('', 'None')]
            print(f"    authored TP/FP/FN      : {int(atp)}/{int(afp)}/{int(afn)}   TN(clean runs): "
                  f"{sum(1 for t in tns if t=='True')}/{len(tns)}")
            print(f"    authored release fires : {int(sum(_nums(rs,'authored_release_false_fires')))}")
            print(f"    authored param accuracy: {_frac(rs,'authored_param_accuracy')}   "
                  f"agent-chosen obs thresholds: {int(sum(_nums(rs,'agent_chosen_obs_thresholds')))}")
        if scn != "S3":
            print(f"    library coverage       : {_frac(rs,'arming_coverage')}   provenance: {_frac(rs,'arming_provenance')}")
        print(f"    monitor false alarms   : {int(sum(_nums(rs,'monitor_false_alarms')))} "
              f"(over {sum(1 for r in rs if str(r.get('monitor_false_alarms','')).strip() not in ('','None'))} clean runs)")
        med = _nums(rs, 'fire_to_next_verdict_median_s', 'fire_to_verdict_median_s')
        mm = _median(med)
        print(f"    fire->next-verdict     : median-of-medians "
              f"{f'{mm}s' if mm is not None else '-'}   "
              f"p95s seen: {_nums(rs,'fire_to_next_verdict_p95_s','fire_to_verdict_p95_s') or '-'}   "
              f"adjudicated {int(sum(_nums(rs,'n_fires_adjudicated')))} / never "
              f"{int(sum(_nums(rs,'n_fires_never_adjudicated')))}")
        print(f"    time_to_verdict        : avg {_avg(_nums(rs,'time_to_verdict_s'))}s")
        warn = sum(1 for r in rs for k in ("label_consistency_warning","scenario_consistency_warning")
                   if str(r.get(k, "")).strip())
        if warn: print(f"    !! rows with warnings  : {warn}")
    print(f"\n{'='*78}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", nargs="?", default=None)
    ap.add_argument("label", nargs="?", default=None,
                    help="label YAML; if omitted, the single *.yaml inside session_dir is used")
    ap.add_argument("--out", default=None)
    ap.add_argument("--csv", default=None, help="results.csv path (default: <class folder>/results.csv)")
    ap.add_argument("--summarize", nargs="+", default=None, metavar="CSV",
                    help="print a per-scenario summary over these results.csv files and exit")
    a = ap.parse_args()
    if a.summarize:
        summarize(a.summarize)
        return
    if not a.session_dir:
        ap.error("session_dir is required (or use --summarize)")
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
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(out); print(f"wrote {a.out}")

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
    print(f"            verdict cells TP={row.get('verdict_TP')} FP={row.get('verdict_FP')} "
          f"TN={row.get('verdict_TN')} FN={row.get('verdict_FN')}  (run-level; separate from fault_*)")
    rc = row.get("recovered_correct")
    print(f"  RECOVERED watchdog={row.get('watchdog_recovered')}  expected={label.get('expected_recovered')}   [{mark(rc) if rc is not None else '-'}]")
    print(f"  CADENCE   one final verdict = {row.get('cadence_compliant')}   [{mark(row.get('cadence_compliant'))}]")
    print(f"  EXECUTOR  terminal_status={row.get('executor_terminal_status')}   independence_gap={row.get('independence_gap')}")

    # detection: expected vs got
    det = row.get("detection", {})
    real = [f"{v.get('step')}:{v.get('object')}" for v in (label.get('violation_steps') or [])]
    print(f"\n  FAULT DETECTION  (step-level pairing, per step:object)")
    print(f"    expected real violations : {real if real else '(none — clean run)'}")
    print(f"    TP (caught)              : {det.get('true_positives') or '-'}")
    fn = det.get('false_negatives'); fp = det.get('false_positives')
    print(f"    FN (MISSED)              : {BAD+str(fn)+X if fn else '-'}")
    print(f"    FP (false alarm)         : {BAD+str(fp)+X if fp else '-'}")
    print(f"    TN (clean steps)         : {row.get('fault_TN')}")
    print(f"    recall={row.get('detection_recall')}  precision={row.get('detection_precision')}")
    if row.get('monitor_false_alarms'):
        print(f"    monitor false alarms (fires on a clean run): {row.get('monitor_false_alarms')}")
    if row.get('authored_probe_fires'):
        print(f"    authored probe/debug fires (excluded from false alarms): "
              f"{row.get('authored_probe_fires')}")

    # authored lane (S2/S3): boolean per injected fault, own columns
    if row.get("authored_detection") is not None:
        ad = row["authored_detection"]
        print(f"\n  AUTHORED MONITORS  (boolean per injected fault; "
              f"{row.get('n_authored_arms')} arms, "
              f"{row.get('n_authored_obs_monitors')} observation-purpose)")
        print(f"    TP={row.get('authored_TP')}  FP={row.get('authored_FP')}  "
              f"FN={row.get('authored_FN')}  TN={row.get('authored_TN')}")
        if ad.get("fn"):
            print(f"    {BAD}missed faults: {ad['fn']}{X}")
        if ad.get("fp"):
            print(f"    {BAD}false fires  : "
                  f"{[(f['object'], f['monitor_id']) for f in ad['fp']]}{X}")
        if row.get("authored_release_false_fires"):
            print(f"    release-time fires (own bucket, not FP): "
                  f"{row.get('authored_release_false_fires')}")
        print(f"    param accuracy={row.get('authored_param_accuracy')}   "
              f"agent-chosen observation thresholds (exempt): "
              f"{row.get('agent_chosen_obs_thresholds')}")
    scn_warn = row.get("scenario_consistency_warning")
    if scn_warn:
        print(f"\n  {BAD}!! SCENARIO WARNING:{X}")
        for wln in scn_warn: print(f"     - {wln}")

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
    print(f"            fire->next-verdict (heuristic pairing): "
          f"median={row.get('fire_to_next_verdict_median_s')}s "
          f"p95={row.get('fire_to_next_verdict_p95_s')}s   "
          f"adjudicated={row.get('n_fires_adjudicated')} "
          f"never={row.get('n_fires_never_adjudicated')}")
    if row.get('n_fires_missing_trigger_stamp'):
        print(f"            {BAD}{row.get('n_fires_missing_trigger_stamp')} fire(s) lack "
              f"trigger_wall_ns (pre-schema session) — skipped, not drain-timed{X}")

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