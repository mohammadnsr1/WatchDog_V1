"""Common-TF-time tick sampling (PREBATCH_FIXES item 2).

The failure this guards against, measured live on the 2026-10-02 S3 bag: the
robot chain republishes slower than the object chains, so two latest-time
(Time(0)) lookups in one recorder tick are evaluated up to ~50 ms apart, and a
relative quantity built from the two samples shows motion-induced phantom
drift (18-26 mm during ordinary moves -- two slip monitors fired on a grasp
that never slipped). The fix: every tick samples all chains at ONE common TF
time, recorded as tf_stamp_ns on each sample.

Here the geometry makes the artifact exact: the tcp chain leads a rigidly
attached object chain by 150 ms while both move at constant velocity along x.
Latest-per-chain sampling reads the rigid 0.100 m separation as 0.130 m
(v * 150 ms = 30 mm of lag error); aligned sampling reads 0.100 m.
"""

from __future__ import annotations

import math

import pytest

try:
    import rclpy.time
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
except ImportError:  # pragma: no cover - ROS not sourced
    pytest.skip("tf2_ros/rclpy not importable; source ROS to run", allow_module_level=True)

from robot_backend.watchdog.recorder import EvidenceRecorder

NS = 1_000_000_000
BASE_NS = 1000 * NS          # an arbitrary TF epoch
VEL = 0.2                    # m/s along +x
SEPARATION = 0.100           # cube rides rigidly 0.100 m behind the tcp (in -x)
LEAD_NS = 150_000_000        # how far the tcp chain's latest runs ahead

REGISTRY = {
    "objects": {
        "cube": {"enabled": True, "tf_frame": "cube", "lookup_frame": "world"},
    },
    "cell": {"base_frame": "world", "gripper": {"tcp_frame": "tcp"}},
}


def _ts(stamp_ns: int, child: str, x: float) -> TransformStamped:
    ts = TransformStamped()
    ts.header.frame_id = "world"
    ts.header.stamp.sec = stamp_ns // NS
    ts.header.stamp.nanosec = stamp_ns % NS
    ts.child_frame_id = child
    ts.transform.translation.x = x
    ts.transform.rotation.w = 1.0
    return ts


def _fill_buffer(extra_child: str | None = None, extra_stamp_ns: int | None = None):
    """tcp keyframes to BASE+600ms; cube keyframes stop 150 ms earlier."""
    buf = tf2_ros.Buffer()
    for step in range(0, 650, 50):
        t = BASE_NS + step * 1_000_000
        seconds = step / 1000.0
        buf.set_transform(_ts(t, "tcp", VEL * seconds), "test")
        if step <= 450:  # cube's chain lags: its latest is tcp's latest - 150 ms
            buf.set_transform(_ts(t, "cube", VEL * seconds - SEPARATION), "test")
    if extra_child is not None:
        buf.set_transform(_ts(extra_stamp_ns, extra_child, 0.0), "test")
    return buf


def _recorder(buf, registry=None):
    recorder = EvidenceRecorder(node=None, registry=registry or REGISTRY)
    recorder.tf_buffer = buf
    return recorder


def _dist(a, b):
    pa, pb = a["pose"]["position"], b["pose"]["position"]
    return math.sqrt(
        (pa["x"] - pb["x"]) ** 2 + (pa["y"] - pb["y"]) ** 2 + (pa["z"] - pb["z"]) ** 2
    )


def test_latest_per_chain_sampling_shows_the_lag_artifact():
    """The OLD method, reproduced inline: 30 mm of phantom separation."""
    buf = _fill_buffer()
    tcp = buf.lookup_transform("world", "tcp", rclpy.time.Time())
    cube = buf.lookup_transform("world", "cube", rclpy.time.Time())
    apparent = abs(tcp.transform.translation.x - cube.transform.translation.x)
    assert apparent == pytest.approx(SEPARATION + VEL * LEAD_NS / NS, abs=1e-6)
    assert apparent - SEPARATION > 0.010  # the artifact exceeds 10 mm


def test_tick_samples_share_one_tf_time_and_show_no_drift():
    recorder = _recorder(_fill_buffer())
    recorder._on_tf_timer()

    tcp = recorder.latest_frame_pose("tcp")
    cube = recorder.latest_object_pose("cube")
    # Both sampled at the cube chain's latest: the latest time available to
    # every chain.
    assert tcp["tf_stamp_ns"] == cube["tf_stamp_ns"] == BASE_NS + 450 * 1_000_000
    # And the rigid separation reads as rigid: ~0 drift.
    assert _dist(tcp, cube) == pytest.approx(SEPARATION, abs=1e-6)


def test_constant_velocity_series_is_driftless_when_aligned():
    recorder = _recorder(_fill_buffer())
    # Several ticks over a growing buffer: separation must never move.
    for end_step in (500, 550, 600):
        recorder._on_tf_timer()
    with recorder.lock:
        tcps = list(recorder.frame_samples["tcp"])
        cubes = list(recorder.object_pose_samples["cube"])
    dists = [
        math.sqrt((t[2] - c[2]) ** 2 + (t[3] - c[3]) ** 2 + (t[4] - c[4]) ** 2)
        for t, c in zip(tcps, cubes)
    ]
    assert max(dists) - min(dists) < 1e-6
    assert dists[0] == pytest.approx(SEPARATION, abs=1e-6)


def test_stale_chain_does_not_drag_the_common_time():
    registry = {
        "objects": {
            "cube": {"enabled": True, "tf_frame": "cube", "lookup_frame": "world"},
            "slowpoke": {"enabled": True, "tf_frame": "slowpoke", "lookup_frame": "world"},
        },
        "cell": {"base_frame": "world", "gripper": {"tcp_frame": "tcp"}},
    }
    stale_ns = BASE_NS - 5 * NS  # 5 s older than everything else
    recorder = _recorder(
        _fill_buffer(extra_child="slowpoke", extra_stamp_ns=stale_ns), registry
    )
    recorder._on_tf_timer()

    tcp = recorder.latest_frame_pose("tcp")
    cube = recorder.latest_object_pose("cube")
    slow = recorder.latest_object_pose("slowpoke")
    # The healthy chains still align at the cube chain's latest...
    assert tcp["tf_stamp_ns"] == cube["tf_stamp_ns"] == BASE_NS + 450 * 1_000_000
    # ...and the stale chain is sampled at its own latest, honestly stamped.
    assert slow["tf_stamp_ns"] == stale_ns


def test_pose_tuple_dict_accepts_both_tuple_shapes():
    recorder = EvidenceRecorder(node=None, registry={"objects": {}})
    old = recorder._pose_tuple_to_dict((1, "cube", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    assert old["tf_stamp_ns"] is None
    new = recorder._pose_tuple_to_dict(
        (1, "cube", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 42)
    )
    assert new["tf_stamp_ns"] == 42
    assert new["wall_ns"] == 1
