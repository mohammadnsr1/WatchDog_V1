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
itself. It must be NOTICED, IDENTIFIED as far as the evidence allows, LOCATED,
and REPORTED with its appearance time. Only an actual requirement break (e.g.
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
  characterization identity text vs ground truth, location error (m), and
                   whether the agent reported WHEN the thing appeared
                   (first_seen_wall_ns vs ground-truth wall_ns).
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
  removal / vanished        -> expect object_missing (only scored when the
                               agent had previously acknowledged the object —
                               you cannot miss the disappearance of something
                               you were never shown to know about; others are
                               reported, not counted)
An episode whose every ground-truth sample has in_camera_view=false is
UNDETECTABLE: reported, excluded from the detection denominator.

Timestamps: ground-truth wall_ns and event-envelope wall_time_iso are both
WALL CLOCK, so they compare across processes. Event payloads' timestamp_ns is
per-process monotonic and is never used here.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from watchdog_scorer import (  # noqa: E402
    _median, _origin, _p95, _purpose, append_csv, load_jsonl,
)

MATCH_RADIUS_M = 0.15        # world-location agreement for a strong obs<->episode match
EPISODE_GAP_S = 30.0         # tracked_move records further apart start a new episode
APPEARANCE_TOL_S = 30.0      # |first_seen claim - ground truth| within this = accurate
PRE_T0_SLACK_S = 2.0         # an obs may precede t0 by this much (clock/record ordering)

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


# ---------------------------------------------------------------------------
# matching: episode -> first acknowledgment
# ---------------------------------------------------------------------------
def _location_error_m(episode, obs):
    loc = obs.get("location") or {}
    if episode["position"] is None or loc.get("frame") != "world":
        return None
    ex, ey = episode["position"][0], episode["position"][1]
    return math.hypot(loc["x"] - ex, loc["y"] - ey)   # planar: z conventions differ


def match_observations(episodes, observations):
    """Assign each episode its FIRST acknowledging observation (status=new or
    update): location within MATCH_RADIUS_M when both sides have a world
    position, else the earliest unassigned category-compatible observation
    after t0. One observation may acknowledge several episodes only via the
    location rule (e.g. one summary covering a move and a settle); the weak
    rule consumes the observation."""
    assigned = [None] * len(episodes)
    used_weak = set()
    for i, ep in enumerate(episodes):
        t0 = ns_to_dt(ep["t0_wall_ns"])
        best = None
        for obs in observations:
            if obs.get("status") == "resolved":
                continue
            ot = iso_to_dt(obs["wall_time_iso"])
            if (ot - t0).total_seconds() < -PRE_T0_SLACK_S:
                continue
            err = _location_error_m(ep, obs)
            strong = err is not None and err <= MATCH_RADIUS_M
            cat_ok = obs.get("category") in EXPECTED_CATEGORY.get(ep["kind"], set())
            weak = (err is None and cat_ok
                    and id(obs) not in used_weak)
            if strong or weak:
                cand = (ot, strong, err, obs)
                if best is None or cand[0] < best[0]:
                    best = cand
        if best is not None:
            ot, strong, err, obs = best
            if not strong:
                used_weak.add(id(obs))
            assigned[i] = {
                "observation_id": obs.get("observation_id"),
                "match": "location" if strong else "category",
                "category": obs.get("category"),
                "latency_s": round((ot - t0).total_seconds(), 2),
                "location_error_m": round(err, 3) if err is not None else None,
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

    row = {
        "run_id": session_dir.name,
        "role": role,
        "n_ground_truth_records": len(gt),
        "n_episodes": len(episodes),
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
    loc_errs = [er["location_error_m"] for er in ep_rows
                if er.get("location_error_m") is not None]
    row["location_error_median_m"] = _median(loc_errs)
    row["appearance_reported_accurate"] = sum(
        1 for er in ep_rows if er.get("appearance_reported") == "accurate")

    # false reports: any observation on a CLEAN run; unmatched ones otherwise
    matched_ids = {a["observation_id"] for a in acks if a}
    unmatched = [o for o in new_obs if o.get("observation_id") not in matched_ids]
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
    "detection_latency_median_s", "detection_latency_p95_s",
    "location_error_median_m", "appearance_reported_accurate",
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
        "detection_latency_median_s": row.get("detection_latency_median_s"),
        "detection_latency_p95_s": row.get("detection_latency_p95_s"),
        "location_error_median_m": row.get("location_error_median_m"),
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
    print(f"              latency median={row['detection_latency_median_s']}s "
          f"p95={row['detection_latency_p95_s']}s   "
          f"loc-err median={row['location_error_median_m']}m   "
          f"appearance-time accurate: {row['appearance_reported_accurate']}")
    for er in row["episodes"]:
        tag = f"{OK}HIT {er.get('latency_s')}s{X}" if er["detected"] else (
            f"{BAD}MISS{X}" if er.get("counted", True) and er["detectable"]
            else "out-of-view" if not er["detectable"] else "uncounted")
        extra = f" fire+{er['fire_latency_s']}s" if er.get("fire_latency_s") is not None else ""
        print(f"    {er['name']:<22} {er['kind']:<22} {tag}{extra}")
    if row.get("false_scene_reports"):
        print(f"  {BAD}FALSE SCENE REPORTS on a clean run: "
              f"{row['false_scene_reports']}{X}")
    if row.get("unmatched_observations"):
        print(f"  unmatched observations (review by hand): "
              f"{len(row['unmatched_observations'])}")
    print("=" * 70)


if __name__ == "__main__":
    sys.exit(main())
