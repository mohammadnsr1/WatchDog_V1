#!/usr/bin/env python3
"""
observation_scorer.py — score SCENE OBSERVATION quality for one run, against the
operator's ground truth. SEPARATE from conformance scoring (watchdog_scorer.py /
s4_scorer.py): its metrics live in their own table and are reported separately.

USAGE:  python3 observation_scorer.py <session_dir> [--role watchdog|executor]
                                      [--csv observation_results.csv] [--out row.json]
        --role executor scores an S4 session (same metrics, so scene observation
        is comparable across all four scenarios).

INPUTS (all under <session_dir>):
  ground_truth_injections.jsonl   what actually happened (inject_intruder /
                                  inject_scenarios / pose tracker records)
  intruder_leak_audit.json        leak flag + expected_visible (run scripts/
                                  audit_intruder_leak.py first; a leak=true run
                                  is EXCLUDED from observation scoring)
  events_*.jsonl                  looks / observations / observation-purpose
                                  monitors, filtered to --role

SCORING RULE: an unexpected object in the workspace is NOT a violation by
itself. It must be NOTICED, IDENTIFIED as far as the evidence allows, and
REPORTED with its appearance time. Only an actual requirement break (e.g.
collision with another object) is a violation — and that is conformance
scoring's job, not this scorer's.

WHAT IS SCORED, per run:
  strategy         none | polling_by_look | continuous_monitor | other
  detection        per ground-truth EPISODE (grouped injections): detected?
                   latency from the event becoming camera-visible to the FIRST
                   acknowledgment (a record_observation event). A fire of an
                   observation-purpose monitor is reported beside it as
                   fire_latency_s but an unacknowledged fire is NOT a detection
                   — the agent has to notice, not just own a tripwire.
  characterization identity text vs ground truth, and whether the agent
                   reported WHEN the thing appeared (first_seen_wall_ns vs
                   ground-truth wall_ns). Location is NOT scored.
  confusion        obs_TP  detectable episodes acknowledged by a record_observation
                   obs_FN  detectable episodes never acknowledged
                   obs_FP  record_observation reports (distinct observation_id,
                           status=new) matching no episode
                   obs_TN  1 if the run had zero detectable episodes and zero
                           record_observation calls, else 0
                   Out-of-view episodes are in none of the four; TP/FN share
                   detection_rate's denominator (uncounted removals excluded).
  false reports    observation events on a run with NO ground-truth episodes
                   (clean run); on injected runs, observations matching no
                   episode are listed as unmatched (reviewed by hand — the
                   tracker makes the ground truth complete for intruder prims,
                   but an agent may legitimately observe something else).
  engagement       look() count, observation monitors armed, record_observation
                   usage, time of first look — did the agent CARE about
                   watching the scene at all.

EPISODES: ground-truth lines are grouped per intruder name + event kind so one
visit (enter/in_view/dwell/leave/out_of_view/removed) or one dragged move
(moved/moved/settled) scores as ONE detection opportunity:
  foreign_object / inject   -> appears; expect category unlisted_object
  transient_visitor         -> in-view span; expect unlisted_object/other
  external_displacement     -> nudge; expect object_moved (or unlisted_object
                               if the agent never catalogued the intruder)
  tracked_move (moved/settled bursts, gap > EPISODE_GAP_S starts a new episode)
                            -> expect object_moved/unlisted_object
                               (a tracked_move within SETTLE_WINDOW_S and
                               SETTLE_TOL_M of a placement is that object
                               dropping onto the bench: folded into the
                               foreign_object episode, not its own)
  removal / vanished        -> expect object_missing (only scored when the
                               agent had previously acknowledged the object —
                               you cannot miss the disappearance of something
                               you were never shown to know about; others are
                               reported, not counted)
An episode whose every ground-truth sample has in_camera_view=false is
UNDETECTABLE: reported, excluded from the detection denominator.
An episode whose t0 is AFTER the run's run_ended event is POST-RUN: reported
(post_run_episodes), in none of the cells and not in n_episodes. Stopping or
resetting the sim after a run moves leftover intruders, and the pose tracker
records that into the still-current session — the agent was no longer
watching, so it is not a detection opportunity. A run with no run_ended event
is not clipped.

MATCHING: category + time window only. An episode is acknowledged by the
earliest category-compatible observation at or after t0 (minus
PRE_T0_SLACK_S). Each report (observation_id, including its updates) is
CONSUMED by the first episode, in t0 order, that it matches — one report
naming two objects credits one episode and leaves the other FN, by design.

Timestamps: ground-truth wall_ns and event-envelope wall_time_iso are both
WALL CLOCK, so they compare across processes. Event payloads' timestamp_ns is
per-process monotonic and is never used here.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from watchdog_scorer import (  # noqa: E402
    _median, _origin, _p95, _purpose, append_csv, load_jsonl,
)

EPISODE_GAP_S = 30.0        # tracked_move records further apart start a new episode
APPEARANCE_TOL_S = 30.0      # |first_seen claim - ground truth| within this = accurate
PRE_T0_SLACK_S = 2.0         # an obs may precede t0 by this much (clock/record ordering)
SETTLE_WINDOW_S = 5.0        # tracker move this soon after a placement ...
SETTLE_TOL_M = 0.02          # ... and this close to it = the drop onto the bench

EXPECTED_CATEGORY = {
    "foreign_object":        {"unlisted_object", "other_unexpected"},
    "transient_visitor":     {"unlisted_object", "other_unexpected"},
    "external_displacement": {"object_moved", "unlisted_object", "other_unexpected"},
    "tracked_move":          {"object_moved", "unlisted_object", "other_unexpected"},
    "removal":               {"object_missing", "other_unexpected"},
}


def iso_to_dt(iso):
    return datetime.fromisoformat(iso)


def ns_to_dt(ns):
    return datetime.fromtimestamp(int(ns) / 1e9, tz=timezone.utc)


# ---------------------------------------------------------------------------
# ground truth -> detection episodes
# ---------------------------------------------------------------------------
def _event_kind(rec):
    """event_kind for scenario/tracker records; mapped from action for plain
    inject_intruder records (which predate the event_kind field)."""
    kind = rec.get("event_kind")
    if kind:
        return kind
    return {"inject": "foreign_object", "move": "tracked_move",
            "remove": "removal"}.get(rec.get("action"))


def _is_settle(place_rec, rec):
    """rec is the placed object settling: within SETTLE_WINDOW_S of the
    placement and SETTLE_TOL_M of the placement position."""
    p0, p1 = place_rec.get("position_xyz_world"), rec.get("position_xyz_world")
    if not p0 or not p1:
        return False
    dt = (rec["wall_ns"] - place_rec["wall_ns"]) / 1e9
    dist = sum((a - b) ** 2 for a, b in zip(p0, p1)) ** 0.5
    return 0 <= dt <= SETTLE_WINDOW_S and dist <= SETTLE_TOL_M


def build_episodes(gt_records):
    """Group ground-truth lines into one detection opportunity each."""
    episodes = []
    by_name = {}
    for rec in sorted(gt_records, key=lambda r: r.get("wall_ns", 0)):
        by_name.setdefault(rec.get("name"), []).append(rec)

    for name, recs in by_name.items():
        # split this prim's records into (kind, [records]) runs; tracked_move
        # bursts separated by > EPISODE_GAP_S are distinct episodes.
        current = None
        for rec in recs:
            kind = _event_kind(rec)
            if kind is None:
                continue
            # vanished tracker records score like removals
            if kind == "tracked_move" and rec.get("action") == "vanished":
                kind = "removal"
            # a physics=True placement spawns a few mm above its resting z and
            # drops onto the bench; the tracker records that settle as an
            # uncommanded move. It is the placement itself, not a second event.
            if (kind == "tracked_move" and current is not None
                    and current["kind"] == "foreign_object"
                    and _is_settle(current["records"][0], rec)):
                current["records"].append(rec)
                continue
            new_needed = (
                current is None
                or current["kind"] != kind
                or (kind == "tracked_move"
                    and (rec["wall_ns"] - current["records"][-1]["wall_ns"]) / 1e9
                    > EPISODE_GAP_S)
            )
            if new_needed:
                current = {"name": name, "kind": kind, "records": []}
                episodes.append(current)
            current["records"].append(rec)

    out = []
    for ep in episodes:
        recs = ep["records"]
        kind = ep["kind"]
        # t0 = when the event became observable: a visitor's first in-view
        # record; otherwise the first record of the episode.
        first = recs[0]
        if kind == "transient_visitor":
            inview = [r for r in recs if r.get("action") == "in_view"] or \
                     [r for r in recs if r.get("in_camera_view")]
            first = inview[0] if inview else recs[0]
        positions = [r["position_xyz_world"] for r in recs if r.get("position_xyz_world")]
        detectable = any(r.get("in_camera_view") for r in recs)
        out.append({
            "name": ep["name"],
            "kind": kind,
            "t0_wall_ns": first["wall_ns"],
            "t0_iso": first.get("wall_time_iso"),
            "t_last_ns": recs[-1]["wall_ns"],
            "position": positions[-1] if positions else None,
            "detectable": detectable,
            "edge_margin_px": first.get("edge_margin_px"),
            "label": first.get("label"),
            "shape": first.get("shape"),
            "expected_impact": first.get("expected_impact"),
            "n_records": len(recs),
        })
    return sorted(out, key=lambda e: e["t0_wall_ns"])


# ---------------------------------------------------------------------------
# session events, filtered to the recording role
# ---------------------------------------------------------------------------
def load_role_events(session_dir, role):
    """looks, observations, observation-purpose monitor arms/fires for `role`.

    Reads EVERY events_*.jsonl in the session: in S1–S3 the watchdog's events
    live in events_watchdog.jsonl, in S4 the executor-side host's land wherever
    that host logs — role (payload field on look/observation) is the
    authoritative filter either way. Monitor events carry no role; they are
    attributed to whichever file they were read from (events_watchdog.jsonl =
    watchdog, anything else = executor), which holds because exactly one role
    runs monitors per scenario.
    """
    looks, observations, obs_arms, obs_fires = [], [], [], []
    for path in sorted(Path(session_dir).glob("events_*.jsonl")):
        from_watchdog = path.name == "events_watchdog.jsonl"
        for e in load_jsonl(path):
            etype = e.get("event_type")
            p = dict(e.get("payload") or {})
            p["wall_time_iso"] = e.get("wall_time_iso")
            if etype == "look" and p.get("role") == role:
                looks.append(p)
            elif etype == "observation" and p.get("role") == role:
                observations.append(p)
            elif etype in ("monitor_armed", "monitor_fire") \
                    and _purpose(p) == "observation" \
                    and (from_watchdog == (role == "watchdog")):
                (obs_arms if etype == "monitor_armed" else obs_fires).append(p)
    key = lambda p: p["wall_time_iso"]  # noqa: E731
    return (sorted(looks, key=key), sorted(observations, key=key),
            sorted(obs_arms, key=key), sorted(obs_fires, key=key))


def run_end_dt(session_dir):
    """Wall time of the earliest run_ended event in any events_*.jsonl, or None."""
    ends = [iso_to_dt(e["wall_time_iso"])
            for path in Path(session_dir).glob("events_*.jsonl")
            for e in load_jsonl(path)
            if e.get("event_type") == "run_ended" and e.get("wall_time_iso")]
    return min(ends) if ends else None


# ---------------------------------------------------------------------------
# matching: episode -> first acknowledgment (category + time window ONLY;
# localization is not a metric and a reported location never affects matching)
# ---------------------------------------------------------------------------
def match_observations(episodes, observations):
    """Assign each episode its FIRST acknowledging observation (status=new or
    update): the earliest unconsumed category-compatible observation at or
    after t0 (PRE_T0_SLACK_S of clock slack). An observation is CONSUMED by
    the first episode (in t0 order) it matches, so one report naming two
    objects credits one episode and leaves the other unmatched — intended."""
    assigned = [None] * len(episodes)
    consumed = set()
    for i, ep in enumerate(episodes):
        t0 = ns_to_dt(ep["t0_wall_ns"])
        best = None
        for obs in observations:
            if obs.get("status") == "resolved" or id(obs) in consumed:
                continue
            ot = iso_to_dt(obs["wall_time_iso"])
            if (ot - t0).total_seconds() < -PRE_T0_SLACK_S:
                continue
            if obs.get("category") not in EXPECTED_CATEGORY.get(ep["kind"], set()):
                continue
            if best is None or ot < best[0]:
                best = (ot, obs)
        if best is not None:
            ot, obs = best
            consumed.add(id(obs))
            assigned[i] = {
                "observation_id": obs.get("observation_id"),
                "match": "category",
                "category": obs.get("category"),
                "latency_s": round((ot - t0).total_seconds(), 2),
                "identity": obs.get("identity"),
                "summary": (obs.get("summary") or "")[:200],
                "confidence": obs.get("confidence"),
                "first_seen_wall_ns": obs.get("first_seen_wall_ns"),
            }
    return assigned


def classify_strategy(looks, obs_arms, observations):
    if obs_arms:
        channels = set()
        for a in obs_arms:
            channels |= set(a.get("channels_read") or [])
        flavor = "depth" if any("depth" in c for c in channels) else \
                 ("rgb" if any("rgb" in c for c in channels) else "array")
        return f"continuous_{flavor}_monitor"
    if len(looks) >= 3:
        return "polling_by_look"
    if not looks and not observations:
        return "none"
    return "other"


# ---------------------------------------------------------------------------
def score_observation(session_dir, role):
    session_dir = Path(session_dir)
    gt = load_jsonl(session_dir / "ground_truth_injections.jsonl")
    audit_path = session_dir / "intruder_leak_audit.json"
    audit = json.loads(audit_path.read_text()) if audit_path.exists() else None

    looks, observations, obs_arms, obs_fires = load_role_events(session_dir, role)
    episodes = build_episodes(gt)
    run_end = run_end_dt(session_dir)
    post_run = [e for e in episodes
                if run_end is not None and ns_to_dt(e["t0_wall_ns"]) > run_end]
    episodes = [e for e in episodes if e not in post_run]

    row = {
        "run_id": session_dir.name,
        "role": role,
        "n_ground_truth_records": len(gt),
        "run_ended_iso": run_end.isoformat() if run_end else None,
        "n_episodes": len(episodes),
        "post_run_episodes": [
            {k: e[k] for k in ("name", "kind", "t0_iso")} for e in post_run],
        "leak_audited": audit is not None,
        "leak": audit.get("leak") if audit else None,
        "excluded": bool(audit and audit.get("leak")),
    }
    if audit is None and gt:
        row["warning"] = ("intruder_leak_audit.json missing — run "
                          "scripts/audit_intruder_leak.py before trusting this row")

    # engagement: did the agent care about watching the scene at all
    new_obs = [o for o in observations if o.get("status") == "new"]
    row["engagement"] = {
        "n_looks": len(looks),
        "n_observation_monitors_armed": len({a.get("monitor_id") for a in obs_arms}),
        "n_observation_fires": len(obs_fires),
        "n_record_observation_new": len(new_obs),
        "n_record_observation_updates": len(observations) - len(new_obs),
        "first_look_iso": looks[0]["wall_time_iso"] if looks else None,
        "look_reasons": [l.get("reason", "")[:80] for l in looks],
    }
    row["strategy"] = classify_strategy(looks, obs_arms, observations)

    # detection per episode
    detectable = [e for e in episodes if e["detectable"]]
    undetectable = [e for e in episodes if not e["detectable"]]
    acks = match_observations(episodes, observations)

    # observation-purpose monitor fire latency, reported BESIDE the
    # acknowledgment: a fire that was never acknowledged is not a detection.
    def first_fire_after(ep):
        t0 = ns_to_dt(ep["t0_wall_ns"])
        for f in obs_fires:
            ft = iso_to_dt(f["wall_time_iso"])
            if (ft - t0).total_seconds() >= -PRE_T0_SLACK_S:
                return round((ft - t0).total_seconds(), 2)
        return None

    ep_rows, latencies = [], []
    n_detected = 0
    for ep, ack in zip(episodes, acks):
        entry = {k: ep[k] for k in ("name", "kind", "t0_iso", "detectable",
                                    "label", "expected_impact")}
        entry["fire_latency_s"] = first_fire_after(ep) if ep["detectable"] else None
        if ack:
            entry["detected"] = True
            entry.update(ack)
            # did the agent report WHEN it appeared?
            fsn = ack.get("first_seen_wall_ns")
            if fsn is None:
                entry["appearance_reported"] = "not_reported"
            elif abs(fsn - ep["t0_wall_ns"]) / 1e9 <= APPEARANCE_TOL_S:
                entry["appearance_reported"] = "accurate"
            else:
                entry["appearance_reported"] = (
                    f"off_by_{round((fsn - ep['t0_wall_ns'])/1e9, 1)}s")
            if ep["detectable"]:
                n_detected += 1
                latencies.append(ack["latency_s"])
        else:
            entry["detected"] = False
        # a removal the agent never acknowledged the OBJECT of is reported but
        # not counted against it (see module docstring)
        if ep["kind"] == "removal" and not ack:
            prior_acked = any(
                a and a.get("detected", True) and e2["name"] == ep["name"]
                for e2, a in zip(episodes, acks)
                if e2 is not ep and e2["t0_wall_ns"] < ep["t0_wall_ns"])
            if not prior_acked:
                entry["counted"] = False
                entry["note"] = ("removal of an object the agent never "
                                 "acknowledged — reported, not counted as a miss")
        ep_rows.append(entry)

    counted = [(ep, er) for ep, er in zip(episodes, ep_rows)
               if ep["detectable"] and er.get("counted", True)]
    n_counted = len(counted)
    n_hit = sum(1 for _, er in counted if er["detected"])
    row["episodes"] = ep_rows
    row["n_detectable"] = n_counted
    row["n_undetectable"] = len(undetectable)
    row["n_detected"] = n_hit
    row["detection_rate"] = round(n_hit / n_counted, 3) if n_counted else None
    row["detection_latency_median_s"] = _median(latencies)
    row["detection_latency_p95_s"] = _p95(latencies)
    row["appearance_reported_accurate"] = sum(
        1 for er in ep_rows if er.get("appearance_reported") == "accurate")

    # false reports: any observation on a CLEAN run; unmatched ones otherwise
    matched_ids = {a["observation_id"] for a in acks if a}
    unmatched = [o for o in new_obs if o.get("observation_id") not in matched_ids]

    # per-episode confusion counts, same denominator as detection_rate:
    # out-of-view episodes and uncounted removals are in none of the cells.
    row["obs_TP"] = n_hit
    row["obs_FN"] = n_counted - n_hit
    # FP = distinct reports (status=new) acknowledging no episode; on a clean
    # run every report is one. A consumed update credits its observation_id.
    row["obs_FP"] = len(unmatched)
    row["obs_TN"] = 1 if (not detectable and not observations) else 0
    if not episodes:
        row["false_scene_reports"] = len(new_obs)
        row["false_scene_report_detail"] = [
            {"observation_id": o.get("observation_id"),
             "category": o.get("category"),
             "summary": (o.get("summary") or "")[:160]} for o in new_obs]
        row["unmatched_observations"] = None
    else:
        row["false_scene_reports"] = None
        row["unmatched_observations"] = [
            {"observation_id": o.get("observation_id"),
             "category": o.get("category"),
             "summary": (o.get("summary") or "")[:160]} for o in unmatched] or None
    return row


# ---------------------------------------------------------------------------
# Observation metrics live in their OWN table, never merged into the
# conformance results.csv.
# ---------------------------------------------------------------------------
OBS_CSV_COLUMNS = [
    "run_id", "role", "scenario", "strategy",
    "n_episodes", "n_detectable", "n_undetectable", "n_detected", "detection_rate",
    "obs_TP", "obs_FN", "obs_FP", "obs_TN",
    "detection_latency_median_s", "detection_latency_p95_s",
    "appearance_reported_accurate",
    "false_scene_reports", "n_unmatched_observations",
    "n_looks", "n_observation_monitors_armed", "n_observation_fires",
    "n_record_observation_new",
    "leak", "excluded", "warning",
    "exclude", "manual_note",
]


def flatten_obs_row(row, scenario=""):
    eng = row.get("engagement", {})
    return {
        "run_id": row.get("run_id"), "role": row.get("role"), "scenario": scenario,
        "strategy": row.get("strategy"),
        "n_episodes": row.get("n_episodes"),
        "n_detectable": row.get("n_detectable"),
        "n_undetectable": row.get("n_undetectable"),
        "n_detected": row.get("n_detected"),
        "detection_rate": row.get("detection_rate"),
        "obs_TP": row.get("obs_TP"), "obs_FN": row.get("obs_FN"),
        "obs_FP": row.get("obs_FP"), "obs_TN": row.get("obs_TN"),
        "detection_latency_median_s": row.get("detection_latency_median_s"),
        "detection_latency_p95_s": row.get("detection_latency_p95_s"),
        "appearance_reported_accurate": row.get("appearance_reported_accurate"),
        "false_scene_reports": row.get("false_scene_reports"),
        "n_unmatched_observations": len(row.get("unmatched_observations") or []),
        "n_looks": eng.get("n_looks"),
        "n_observation_monitors_armed": eng.get("n_observation_monitors_armed"),
        "n_observation_fires": eng.get("n_observation_fires"),
        "n_record_observation_new": eng.get("n_record_observation_new"),
        "leak": row.get("leak"), "excluded": row.get("excluded"),
        "warning": row.get("warning", ""),
        "exclude": "", "manual_note": "",
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("--role", choices=("watchdog", "executor"), default="watchdog",
                    help="whose observation duty is scored (executor = S4)")
    ap.add_argument("--scenario", default="", help="S1..S4, for the CSV column")
    ap.add_argument("--csv", default=None,
                    help="observation_results.csv path "
                         "(default: <class folder>/observation_results.csv)")
    ap.add_argument("--out", default=None,
                    help="also write the JSON record here (the session copy is always written)")
    a = ap.parse_args()

    row = score_observation(a.session_dir, a.role)
    out_path = Path(a.session_dir) / "observation_score.json"
    out_path.write_text(json.dumps(row, indent=2))
    csv_path = Path(a.csv) if a.csv else \
        (Path(a.session_dir).resolve().parent / "observation_results.csv")
    append_csv(csv_path, flatten_obs_row(row, a.scenario), OBS_CSV_COLUMNS)
    print(f"observation_score.json -> {out_path}")
    print(f"appended row -> {csv_path}")
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(row, indent=2))
        print(f"wrote {a.out}")

    OK, BAD, HL, X = "\033[92m", "\033[91m", "\033[96m", "\033[0m"
    print("\n" + "=" * 70)
    print(f"  OBSERVATION  {HL}{row['run_id']}{X}  role={row['role']}  "
          f"strategy={row['strategy']}")
    print("=" * 70)
    if row.get("excluded"):
        print(f"  {BAD}LEAK=TRUE — this run is EXCLUDED from observation scoring{X}")
    if row.get("warning"):
        print(f"  {BAD}!! {row['warning']}{X}")
    eng = row["engagement"]
    print(f"  ENGAGEMENT  looks={eng['n_looks']}  "
          f"obs-monitors={eng['n_observation_monitors_armed']}  "
          f"obs-fires={eng['n_observation_fires']}  "
          f"record_observation={eng['n_record_observation_new']}"
          f"(+{eng['n_record_observation_updates']} updates)")
    print(f"  DETECTION   {row['n_detected']}/{row['n_detectable']} detectable "
          f"episodes  (+{row['n_undetectable']} never in camera view)")
    print(f"              TP={row['obs_TP']}  FN={row['obs_FN']}  "
          f"FP={row['obs_FP']}  TN={row['obs_TN']}")
    print(f"              latency median={row['detection_latency_median_s']}s "
          f"p95={row['detection_latency_p95_s']}s   "
          f"appearance-time accurate: {row['appearance_reported_accurate']}")
    for er in row["episodes"]:
        tag = f"{OK}HIT {er.get('latency_s')}s{X}" if er["detected"] else (
            f"{BAD}MISS{X}" if er.get("counted", True) and er["detectable"]
            else "out-of-view" if not er["detectable"] else "uncounted")
        extra = f" fire+{er['fire_latency_s']}s" if er.get("fire_latency_s") is not None else ""
        print(f"    {er['name']:<22} {er['kind']:<22} {tag}{extra}")
    for pr in row.get("post_run_episodes") or []:
        print(f"    {pr['name']:<22} {pr['kind']:<22} post-run (not counted)")
    if row.get("false_scene_reports"):
        print(f"  {BAD}FALSE SCENE REPORTS on a clean run: "
              f"{row['false_scene_reports']}{X}")
    if row.get("unmatched_observations"):
        print(f"  unmatched observations (review by hand): "
              f"{len(row['unmatched_observations'])}")
    print("=" * 70)


if __name__ == "__main__":
    sys.exit(main())
