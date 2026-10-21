"""watchdog_scorer / observation_scorer helpers (the repo-root scripts).

Pins the 2026-10-16 scoring revision: the purpose="observation" exemption is
decided by what a monitor READS (camera-only stays exempt; a pose / joint /
frame monitor flagged observation scores as a clause monitor), coverage
credits such a monitor, and the observation strategy label tells a pose-only
monitor (blind to anything unregistered) apart from a camera one.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

import observation_scorer as obs  # noqa: E402
import watchdog_scorer as wd  # noqa: E402


def _arm(purpose, channels=None, sha=None, **extra):
    p = {"origin": "authored", "purpose": purpose, "monitor_id": "m"}
    if channels is not None:
        p["channels_read"] = channels
    if sha:
        p["code_sha256"] = sha
    p.update(extra)
    return p


def test_camera_only_observation_monitor_is_exempt():
    assert wd._is_observation(_arm("observation", ["rgb"]))
    assert wd._is_observation(_arm("observation", ["depth", "camera"]))


def test_pose_joint_frame_reads_score_as_clause_whatever_the_flag():
    assert not wd._is_observation(_arm("observation", ["object_pose:u_orange"]))
    assert not wd._is_observation(_arm("observation", ["joint:finger_joint"]))
    assert not wd._is_observation(_arm("observation", ["transform:tcp->u_orange"]))
    assert not wd._is_observation(_arm("observation", ["frame_pose_window:tcp"]))
    # a camera monitor that ALSO reads a pose is judging a requirement
    assert not wd._is_observation(_arm("observation", ["rgb", "object_pose:base1"]))


def test_clause_purpose_is_never_exempt():
    assert not wd._is_observation(_arm("clause", ["rgb"]))
    assert not wd._is_observation({"origin": "authored"})   # purpose defaults to clause


def test_missing_channels_fall_back_to_the_saved_source():
    src = {"abc": "def check(ctx):\n    p = ctx.object_pose('u_orange')\n"}
    assert not wd._is_observation(_arm("observation", None, "abc"), src)
    assert wd._is_observation(_arm("observation", None, "zzz"), src)   # nothing known: exempt


def test_coverage_credits_a_pose_monitor_flagged_observation():
    p = _arm("observation", ["object_pose:u_orange"], object_id="u_orange")
    assert wd._coverage_kinds(p, {}) == [("u_orange", "position_static")]
    assert wd._coverage_kinds(_arm("observation", ["rgb"]), {}) == []


def test_reclassified_count_reaches_the_csv():
    assert "obs_flag_on_clause_reads" in wd.CSV_COLUMNS
    assert wd.flatten_row({"obs_flag_on_clause_reads": 12})["obs_flag_on_clause_reads"] == 12


def test_strategy_label_by_channel():
    assert obs.classify_strategy([], [_arm("observation", ["rgb"])], []) \
        == "continuous_rgb_monitor"
    assert obs.classify_strategy([], [_arm("observation", ["depth"])], []) \
        == "continuous_depth_monitor"
    pose_only = [_arm("observation", ["object_pose:base1"]),
                 _arm("observation", ["joint:finger_joint"])]
    assert obs.classify_strategy([], pose_only, []) == "continuous_pose_monitor"
    assert obs.classify_strategy([], [_arm("observation", [])], []) \
        == "continuous_array_monitor"
    assert obs.classify_strategy([{}, {}, {}], [], []) == "polling_by_look"
    assert obs.classify_strategy([], [], []) == "none"


def test_tp_mode_columns_reach_the_csv_next_to_the_fault_cells():
    cols = wd.CSV_COLUMNS
    assert cols.index("fault_TP_by_fire") == cols.index("fault_FN_list") + 1
    assert cols.index("fault_TP_by_check") == cols.index("fault_TP_by_fire") + 1
    flat = wd.flatten_row({"fault_TP_by_fire": 2, "fault_TP_by_check": 1})
    assert (flat["fault_TP_by_fire"], flat["fault_TP_by_check"]) == (2, 1)


def test_multi_object_fire_subject_comes_from_its_detail():
    chans = ["object_pose:base1", "object_pose:u_brown", "object_pose:u_orange",
             "object_pose:inverted_u_brown"]
    p = {"origin": "authored", "object_id": "base1", "channels_read": chans,
         "detail": "u_orange moved from last-known-rest baseline while not held (held=u_brown)"}
    wd._resolve_multi_object_fire(p)
    assert p["object_id"] == "u_orange" and p["object_id_armed"] == "base1"
    q = {"origin": "authored", "object_id": "base1", "channels_read": chans,
         "detail": "inverted_u_brown displaced"}
    wd._resolve_multi_object_fire(q)
    assert q["object_id"] == "inverted_u_brown"          # whole-word: not u_brown
    single = {"origin": "authored", "object_id": "u_brown",
              "channels_read": ["object_pose:u_brown"], "detail": "u_orange moved"}
    wd._resolve_multi_object_fire(single)
    assert single["object_id"] == "u_brown"              # single-object: untouched
    lib = {"origin": "library", "object_id": "base1", "channels_read": chans,
           "detail": "u_orange moved"}
    wd._resolve_multi_object_fire(lib)
    assert lib["object_id"] == "base1"                   # library: untouched


def test_unresolved_multi_object_threshold_is_checked_against_the_registry():
    reg = {"u_brown": {"spec": {"tolerances": {"disturbance_tolerance_m": 0.02}}},
           "u_orange": {"spec": {"tolerances": {"disturbance_tolerance_m": 0.02}}},
           "base1": {"spec": {"tolerances": {}}}}                 # fixture: no field, skipped
    a = {"threshold": 0.02, "threshold_source": "object_unresolved",
         "threshold_registry_field": "raw_spec.tolerances.disturbance_tolerance_m",
         "channels_read": ["held_object", "object_pose:u_brown", "object_pose:u_orange",
                           "object_pose:base1"]}
    assert wd._verify_unresolved_multi(a, reg)
    assert not wd._verify_unresolved_multi(dict(a, threshold=0.01), reg)     # off the registry
    only_fixture = dict(a, channels_read=["object_pose:base1"])
    assert not wd._verify_unresolved_multi(only_fixture, reg)                # nothing to check
    resolved = dict(a, threshold_source="raw_spec.tolerances.disturbance_tolerance_m")
    assert not wd._verify_unresolved_multi(resolved, reg)                    # server already judged


def test_structural_threshold_uses_the_field_named_in_its_declared_source():
    reg = {"u_green": {"spec": {"tolerances": {"disturbance_tolerance_m": 0.02}}}}
    a = {"threshold": 0.02, "threshold_source": "structural", "object_id": "u_green",
         "threshold_registry_field": None, "channels_read": ["object_pose:u_green"],
         "params": {"disturbance_tolerance_m": {"value": 0.02, "source":
             "u_brown.raw_spec.tolerances.disturbance_tolerance_m (applied to bystander u_green)"}}}
    assert wd._verify_unresolved_multi(a, reg)
    assert not wd._verify_unresolved_multi(dict(a, threshold=0.005), reg)
    no_field = dict(a, params={"tol": {"value": 0.02, "source": "agent guess"}})
    assert not wd._verify_unresolved_multi(no_field, reg)


def test_s4_reverifies_thresholds_against_the_true_registry():
    reg = {"u_brown": {"spec": {"limits": {"max_clamp_effort": 10.0}}}}
    # an S4 executor that bound its corrupted copy's value and "verified" it
    corrupted = {"origin": "authored", "object_id": "u_brown", "threshold": 100.0,
                 "threshold_verified": True,
                 "threshold_registry_field": "raw_spec.limits.max_clamp_effort"}
    wd._reverify_against_registry(corrupted, reg)
    assert corrupted["threshold_verified"] is False
    assert corrupted["executor_claimed_verified"] is True
    correct = dict(corrupted, threshold=10.0, threshold_verified=False)
    correct.pop("executor_claimed_verified")
    wd._reverify_against_registry(correct, reg)
    assert correct["threshold_verified"] is True
    library = dict(corrupted, origin="library", threshold_verified=True)
    wd._reverify_against_registry(library, reg)
    assert library["threshold_verified"] is True        # library arms untouched


def test_s4_csv_is_results_csv_plus_s4_columns():
    import s4_scorer as s4
    shared = [c for c in wd.CSV_COLUMNS if c not in ("exclude", "manual_note")]
    assert s4.S4_CSV_COLUMNS[:len(shared)] == shared
    assert s4.S4_CSV_COLUMNS[-2:] == ["exclude", "manual_note"]
    assert "executor_self_reported" in s4.S4_CSV_COLUMNS
