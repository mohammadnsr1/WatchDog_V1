from __future__ import annotations

from types import SimpleNamespace

import pytest

from robot_backend.watchdog.monitors import (
    EffortEventMonitor,
    RegionMonitor,
    RelativeFrameMonitor,
    StationarityMonitor,
)


class FakeRecorder:
    def __init__(self):
        self.transforms = {("world", "tcp"): {"translation": {"x": 0.0, "y": 0.0, "z": 0.0}}}
        self.poses = {
            "cube": {
                "wall_ns": 10,
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}},
            }
        }

    def lookup_transform(self, parent, child):
        return self.transforms[(parent, child)]

    def latest_object_pose(self, object_id):
        return self.poses.get(object_id)


def joint_state(effort):
    return SimpleNamespace(name=["finger_joint"], effort=[effort])


def test_relative_frame_monitor_fires_on_delta():
    recorder = FakeRecorder()
    monitor = RelativeFrameMonitor(
        {"parent_frame": "world", "child_frame": "tcp", "max_delta_m": 0.05},
        recorder=recorder,
    )
    assert monitor.evaluate(recorder) is None
    recorder.transforms[("world", "tcp")] = {"translation": {"x": 0.06, "y": 0.0, "z": 0.0}}
    assert monitor.evaluate(recorder) == pytest.approx(0.06)


def test_stationarity_monitor_fires_on_displacement():
    recorder = FakeRecorder()
    monitor = StationarityMonitor(
        {"object_id": "cube", "epsilon_m": 0.01}, recorder=recorder
    )
    recorder.poses["cube"] = {
        "wall_ns": 11,
        "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.005}},
    }
    assert monitor.evaluate(recorder) is None
    recorder.poses["cube"] = {
        "wall_ns": 12,
        "pose": {"position": {"x": 0.02, "y": 0.0, "z": 0.0}},
    }
    assert monitor.evaluate(recorder) == pytest.approx(0.02)


def test_region_monitor_modes():
    inside = {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}}
    outside = {"pose": {"position": {"x": 0.2, "y": 0.0, "z": 0.0}}}
    stay_inside = RegionMonitor(
        {
            "object_id": "cube",
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
            "half_extents": {"x": 0.1, "y": 0.1, "z": 0.1},
            "mode": "stay_inside",
        },
    )
    assert stay_inside.evaluate(inside) is None
    assert stay_inside.evaluate(outside) == pytest.approx(0.1)

    stay_outside = RegionMonitor(
        {
            "object_id": "cube",
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
            "half_extents": {"x": 0.1, "y": 0.1, "z": 0.1},
            "mode": "stay_outside",
        },
    )
    assert stay_outside.evaluate(outside) is None
    assert stay_outside.evaluate(inside) == pytest.approx(-0.1)


def test_effort_event_monitor_debounce_and_reset():
    # comparison/threshold express the condition that must HOLD ("effort <=
    # 0.5" is conformant); the monitor fires once that condition is violated
    # on sustained_ticks consecutive samples.
    monitor = EffortEventMonitor(
        {
            "joint_name": "finger_joint",
            "comparison": "le",
            "threshold": 0.5,
            "sustained_ticks": 3,
        },
    )
    assert monitor.evaluate(joint_state(0.6)) is None
    assert monitor.evaluate(joint_state(0.7)) is None
    assert monitor.evaluate(joint_state(0.1)) is None
    assert monitor.evaluate(joint_state(0.6)) is None
    assert monitor.evaluate(joint_state(0.6)) is None
    assert monitor.evaluate(joint_state(0.6)) == pytest.approx(0.6)


def test_effort_event_monitor_conformant_never_fires():
    monitor = EffortEventMonitor(
        {
            "joint_name": "finger_joint",
            "comparison": "le",
            "threshold": 4.0,
            "sustained_ticks": 3,
        },
    )
    assert monitor.evaluate(joint_state(0.0)) is None
    assert monitor.evaluate(joint_state(2.0)) is None
    assert monitor.evaluate(joint_state(3.9)) is None
    assert monitor.evaluate(joint_state(4.5)) is None
    assert monitor.evaluate(joint_state(4.5)) is None
    assert monitor.evaluate(joint_state(4.5)) == pytest.approx(4.5)


def test_param_validation_errors_are_precise():
    with pytest.raises(ValueError, match="epsilon_m must be > 0"):
        StationarityMonitor(
            {"object_id": "cube", "epsilon_m": 0},
            recorder=FakeRecorder(),
        )
    with pytest.raises(ValueError, match="comparison must be one of"):
        EffortEventMonitor(
            {"joint_name": "finger_joint", "comparison": "eq", "threshold": 1.0},
        )
