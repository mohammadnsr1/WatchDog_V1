from __future__ import annotations

from robot_backend.watchdog.recorder import EvidenceRecorder


def make_recorder():
    return EvidenceRecorder(node=None, registry={"objects": {}})


def test_joint_window_bounds_filter_and_downsamples():
    recorder = make_recorder()
    with recorder.lock:
        for i in range(10):
            recorder.joint_samples.append(
                (i, {"j1": float(i), "j2": float(i * 2)}, {"j1": float(i + 10)})
            )

    rows = recorder.get_joint_window(2, 8, ["j1"], max_samples=4)

    assert len(rows) == 4
    assert rows[0]["wall_ns"] == 2
    assert rows[-1]["wall_ns"] == 8
    assert set(rows[0]["positions"]) == {"j1"}
    assert set(rows[0]["efforts"]) == {"j1"}


def test_object_pose_window_bounds_and_downsamples():
    recorder = make_recorder()
    with recorder.lock:
        recorder.object_pose_samples["cube"] = (
            recorder.object_pose_samples.get("cube")
            or type(recorder.joint_samples)(maxlen=100)
        )
        for i in range(10):
            recorder.object_pose_samples["cube"].append(
                (i, "cube", i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            )

    rows = recorder.get_object_pose_window(1, 9, "cube", max_samples=3)

    assert [row["wall_ns"] for row in rows] == [1, 5, 9]
    assert rows[1]["pose"]["position"]["x"] == 5.0


def test_events_window_uses_wall_receive_bounds():
    recorder = make_recorder()
    with recorder.lock:
        for i in range(5):
            recorder.events.append(
                {"wall_ns_received": i, "event_type": "begin_step", "envelope": {"i": i}}
            )

    rows = recorder.get_events_window(1, 3)

    assert [row["wall_ns_received"] for row in rows] == [1, 2, 3]


def test_latest_samples_are_json_safe():
    recorder = make_recorder()
    with recorder.lock:
        recorder.joint_samples.append((100, {"j": 1.0}, {"j": 2.0}))
        recorder.object_pose_samples["cube"] = type(recorder.joint_samples)(
            [(200, "cube", 1, 2, 3, 0, 0, 0, 1)], maxlen=10
        )

    assert recorder.latest_joint_state() == {
        "wall_ns": 100,
        "positions": {"j": 1.0},
        "efforts": {"j": 2.0},
    }
    assert recorder.latest_object_pose("cube")["pose"]["position"] == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
    }
