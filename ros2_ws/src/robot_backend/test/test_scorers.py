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
