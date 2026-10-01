"""The camera array lane: accessors, scheduling, budgets, and purpose.

Everything runs against a fake CameraStreams serving synthetic 360x640 frames
-- no ROS. The engine's real array-lane thread is exercised where scheduling
or isolation is the thing under test; direct evaluate() calls cover the
per-call contracts.
"""

from __future__ import annotations

import os
import time

import numpy as np
import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

from robot_backend.watchdog.authored import (  # noqa: E402
    ARRAY_STATE_MAX_BYTES,
    AuthoredBudgetExceeded,
    AuthoredCodeError,
    AuthoredMonitor,
    AuthoredReturnError,
    json_safe,
)
from robot_backend.watchdog.engine import (  # noqa: E402
    ARRAY_WALL_BUDGET_MS,
    _AUTHORED_STRIKE_LIMIT,
    MonitorEngine,
)
from robot_backend.watchdog.monitors import StationarityMonitor  # noqa: E402
from robot_backend.watchdog.timebase import wall_ns  # noqa: E402

K_FLAT = [554.2563, 0.0, 320.0, 0.0, 554.2563, 180.0, 0.0, 0.0, 1.0]


class FakeArrayStreams:
    """Synthetic camera source with a controllable depth plane and delay."""

    def __init__(self):
        self.depth_value = 1.70
        self.snapshot_sleep = 0.0
        self.snapshots = 0
        self.serve_frames = True

    def array_snapshot(self):
        if self.snapshot_sleep:
            time.sleep(self.snapshot_sleep)
        self.snapshots += 1
        now = wall_ns()
        out = {"taken_wall_ns": now, "rgb": None, "depth": None, "camera": {
            "width": 640, "height": 360, "K": list(K_FLAT),
            "frame_id": "camera_depth_optical_frame", "D": [],
            "distortion_model": "",
            "extrinsics_from_yaml": {
                "xyz": [0.25, -0.2, 1.7],
                "quat_xyzw": [0.0, 0.99863, -0.05234, 0.0],
            },
        }}
        if self.serve_frames:
            rgb = np.zeros((360, 640, 3), dtype=np.uint8)
            rgb.flags.writeable = False
            depth = np.full((360, 640), self.depth_value, dtype=np.float32)
            depth.flags.writeable = False
            out["rgb"] = (rgb, now)
            out["depth"] = (depth, now)
        return out


class FakeRecorder:
    def object_ids(self):
        return ["u_brown"]

    def frame_ids(self):
        return []

    def latest_object_pose(self, object_id):
        return {
            "wall_ns": 0, "object_id": object_id,
            "pose": {"position": dict(self.position),
                     "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        }

    position = {"x": 0.0, "y": 0.0, "z": 0.0}

    def latest_frame_pose(self, frame_id):
        return None

    def latest_joint_state(self):
        return None

    def lookup_transform(self, parent, child):
        raise RuntimeError("no TF in this fake")

    def get_joint_window(self, *a):
        return []

    def get_object_pose_window(self, *a):
        return []

    def get_frame_window(self, *a):
        return []


class FakeBus:
    def __init__(self):
        self.events = []

    def publish(self, event):
        self.events.append(event)

    def payloads(self, event_type):
        return [
            e.model_dump(mode="json")["payload"]
            for e in self.events
            if e.event_type == event_type
        ]


def _wait_until(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _monitor(source, streams, engine=None, rate_hz=2.0, **kwargs):
    return AuthoredMonitor(
        source=source,
        description=kwargs.pop("description", "array test"),
        params=kwargs.pop("params", {}),
        recorder=FakeRecorder(),
        engine=engine if engine is not None else MonitorEngine(FakeBus()),
        camera=streams,
        rate_hz=rate_hz,
        **kwargs,
    )


DEPTH_CHANGE = (
    "def check(ctx):\n"
    "    d = ctx.depth()\n"
    "    if d is None:\n"
    "        return None\n"
    "    m = float(np.nanmean(d))\n"
    "    if 'ref' not in ctx.state:\n"
    "        ctx.state['ref'] = m\n"
    "        return None\n"
    "    delta = abs(m - ctx.state['ref'])\n"
    "    if delta > ctx.value('change_m'):\n"
    "        return {'value': delta, 'detail': 'depth changed'}\n"
    "    return None\n"
)


# --- accessors and snapshot semantics -------------------------------------------

def test_depth_monitor_fires_on_change_and_not_before():
    streams = FakeArrayStreams()
    monitor = _monitor(
        DEPTH_CHANGE, streams,
        params={"change_m": {"value": 0.05, "source": None}},
    )
    assert "depth" in monitor.channels_read
    assert monitor.evaluate(None) is None          # unchanged scene: no fire
    streams.depth_value = 1.42                     # something entered the ROI
    fired = monitor.evaluate(None)
    assert fired == pytest.approx(0.28, abs=1e-6)
    assert monitor.last_detail == "depth changed"
    assert monitor.last_wall_ms is not None


def test_snapshot_is_identical_within_one_call_and_read_only():
    streams = FakeArrayStreams()
    source = (
        "def check(ctx):\n"
        "    a = ctx.rgb()\n"
        "    b = ctx.rgb()\n"
        "    d1 = ctx.depth()\n"
        "    d2 = ctx.depth()\n"
        "    ctx.state['same_rgb'] = a is b\n"
        "    ctx.state['same_depth'] = d1 is d2\n"
        "    return None\n"
    )
    monitor = _monitor(source, streams)
    assert monitor.ctx.state == {"same_rgb": True, "same_depth": True}
    # One snapshot per call, not one per accessor.
    assert streams.snapshots == 1
    monitor.evaluate(None)
    assert streams.snapshots == 2
    # The frames handed to authored code are read-only.
    rgb = monitor.ctx.rgb()
    assert rgb is not None and not rgb.flags.writeable
    with pytest.raises((ValueError, RuntimeError)):
        rgb[0, 0, 0] = 1


def test_absent_frames_return_none_and_camera_shape():
    streams = FakeArrayStreams()
    streams.serve_frames = False
    source = (
        "def check(ctx):\n"
        "    ctx.state['rgb_none'] = ctx.rgb() is None\n"
        "    ctx.state['depth_none'] = ctx.depth() is None\n"
        "    cam = ctx.camera()\n"
        "    ctx.state['fx'] = cam['K'][0][0]\n"
        "    ctx.state['tz'] = cam['world_T_camera']['translation'][2]\n"
        "    return None\n"
    )
    monitor = _monitor(source, streams)
    assert monitor.ctx.state["rgb_none"] is True
    assert monitor.ctx.state["depth_none"] is True
    assert monitor.ctx.state["fx"] == pytest.approx(554.2563)
    assert monitor.ctx.state["tz"] == pytest.approx(1.7)


def test_deproject_scalar_and_vectorised():
    streams = FakeArrayStreams()
    monitor = _monitor(PASSIVE_ARRAY, streams)
    point = monitor.ctx.deproject(320, 180, 1.0)
    assert set(point) == {"x", "y", "z"}
    pts = monitor.ctx.deproject(
        np.array([320, 320]), np.array([180, 180]), np.array([1.0, 1.0])
    )
    assert pts.shape == (2, 3)
    # The optical axis point at both calls is the same world point.
    assert pts[0] == pytest.approx([point["x"], point["y"], point["z"]])
    assert "deproject" in monitor.channels_read


def test_frame_stamps_reports_the_snapshot():
    streams = FakeArrayStreams()
    monitor = _monitor(PASSIVE_ARRAY, streams)
    monitor.ctx._begin_camera_tick()
    stamps = monitor.ctx.frame_stamps()
    assert stamps["rgb_wall_ns"] is not None
    assert stamps["depth_wall_ns"] is not None
    assert stamps["age_ms"] is not None and stamps["age_ms"] >= 0.0


PASSIVE_ARRAY = "def check(ctx):\n    return None\n"


def test_camera_accessor_on_scalar_lane_names_rate_hz():
    streams = FakeArrayStreams()
    with pytest.raises(AuthoredCodeError) as exc_info:
        AuthoredMonitor(
            source="def check(ctx):\n    return ctx.rgb()\n",
            description="scalar", params={},
            recorder=FakeRecorder(), engine=MonitorEngine(FakeBus()),
            camera=streams, rate_hz=None,
        )
    assert exc_info.value.stage == "first_call"
    assert "rate_hz" in str(exc_info.value)


def test_np_is_provided_only_on_the_array_lane():
    with pytest.raises(AuthoredCodeError) as exc_info:
        AuthoredMonitor(
            source="def check(ctx):\n    return None if np else None\n",
            description="scalar", params={},
            recorder=FakeRecorder(), engine=MonitorEngine(FakeBus()),
            camera=None, rate_hz=None,
        )
    assert exc_info.value.stage == "first_call"   # NameError: np
    # And the array lane has it.
    monitor = _monitor(
        "def check(ctx):\n    ctx.state['ok'] = np.zeros(3).shape[0]\n    return None\n",
        FakeArrayStreams(),
    )
    assert monitor.ctx.state["ok"] == 3


NUMPY_BATTERY = (
    "def check(ctx):\n"
    "    d = ctx.depth()\n"
    "    if d is None:\n"
    "        return None\n"
    "    b1 = d > 1.0\n"
    "    b2 = d < 2.0\n"
    "    ok = [float(d.sum()), float(d.mean()), float(d.std()),\n"
    "          int(b1.any()), int(b1.all()), float(d.max()),\n"
    "          float(d.astype('float64')[0, 0]),\n"
    "          int((b1 | b2)[0, 0]), int((b1 & b2)[0, 0]),\n"
    "          int((~b1)[0, 0]), int((b1 ^ b2)[0, 0]),\n"
    "          float(np.nanmean(d)), float(np.nanmedian(d)),\n"
    "          float(np.median(d)), float(np.percentile(d, 50)),\n"
    "          float(np.nanpercentile(d, 50)), len(np.histogram(d)[0]),\n"
    "          len(np.argwhere(b1)), float(np.where(b1, 1.0, 0.0)[0, 0]),\n"
    "          float(np.abs(d)[0, 0]), int(np.isnan(d)[0, 0]),\n"
    "          float(np.clip(d, 0, 2)[0, 0]), len(np.unique(d)),\n"
    "          float(np.cumsum(d[0])[0]),\n"
    "          len(np.convolve(d[0, :3], d[0, :2]))]\n"
    "    total = 0\n"
    "    for _ in range(3):\n"
    "        total = total + 1\n"
    "    ctx.state['battery'] = len(ok) + total\n"
    "    return None\n"
)


def test_numpy_battery_runs_in_the_sandbox():
    """Every listed numpy operation works inside an array-lane check --
    the restricted-builtins lazy-import failure class is gone."""
    streams = FakeArrayStreams()
    monitor = _monitor(NUMPY_BATTERY, streams)
    assert monitor.ctx.state["battery"] == 28
    assert monitor.evaluate(None) is None  # and again, post-construction


def test_forwarded_rgb_frame_keeps_its_header_stamp():
    """The manager's forward into CameraStreams carries the header stamp."""
    from types import SimpleNamespace

    from robot_backend.watchdog.image_buffer import (
        CameraStreams, ImageBufferManager, ImageRingBuffer,
    )

    streams = CameraStreams()
    manager = ImageBufferManager(
        ImageRingBuffer(max_frames=4, max_bytes=1 << 22), streams=streams
    )
    cb = manager._make_callback("/camera/depth/rgb", "raw")
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=345)),
        encoding="rgb8", width=4, height=2, data=bytes(4 * 2 * 3),
    )
    cb(msg)
    got = streams.get_rgb()
    assert got is not None
    _array, stamp_ns = got
    assert stamp_ns == 12 * 1_000_000_000 + 345


# --- budgets ---------------------------------------------------------------------

def test_python_pixel_loop_hits_the_line_budget():
    streams = FakeArrayStreams()
    source = (
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    d = ctx.depth()\n"
        "    total = 0.0\n"
        "    for r in range(d.shape[0]):\n"
        "        for c in range(d.shape[1]):\n"
        "            total = total + d[r, c]\n"
        "    return None\n"
    )
    monitor = _monitor(source, streams)
    with pytest.raises(AuthoredBudgetExceeded):
        monitor.evaluate(None)


def test_state_over_the_array_cap_is_a_bad_return():
    streams = FakeArrayStreams()
    source = (
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    ctx.state['big'] = np.zeros((3000, 3000))\n"
        "    return None\n"
    )
    monitor = _monitor(source, streams)
    with pytest.raises(AuthoredReturnError) as exc_info:
        monitor.evaluate(None)
    assert str(ARRAY_STATE_MAX_BYTES) in str(exc_info.value)


def test_json_safe_summarises_ndarrays():
    arr = np.full((4, 5), 2.5, dtype=np.float32)
    arr[0, 0] = np.nan
    out = json_safe({"frame": arr})
    assert out["frame"]["ndarray"] == [4, 5]
    assert out["frame"]["dtype"] == "float32"
    assert out["frame"]["min"] == pytest.approx(2.5)
    assert out["frame"]["max"] == pytest.approx(2.5)
    assert out["frame"]["nan_frac"] == pytest.approx(0.05)


def test_reference_values_summarise_a_stored_frame():
    streams = FakeArrayStreams()
    source = (
        "def check(ctx):\n"
        "    if 'ref' not in ctx.state:\n"
        "        ctx.state['ref'] = ctx.depth()\n"
        "    return None\n"
    )
    monitor = _monitor(source, streams)
    ref = monitor.reference_values["ref"]
    assert ref["ndarray"] == [360, 640]
    assert "float32" in ref["dtype"]


# --- the array lane thread --------------------------------------------------------

def test_wall_budget_strikes_disarm_and_other_lanes_keep_running():
    bus = FakeBus()
    engine = MonitorEngine(bus)
    recorder = FakeRecorder()
    engine.attach_recorder(recorder)
    streams = FakeArrayStreams()
    streams.snapshot_sleep = 0.2  # the accessor sleeps INSIDE check's window
    slow = _monitor(
        "def check(ctx):\n    d = ctx.depth()\n    return None\n",
        streams, engine=engine, rate_hz=2.0,
    )
    streams.snapshot_sleep = 0.2
    library = StationarityMonitor(
        {"object_id": "u_brown", "epsilon_m": 0.02},
        recorder=recorder, monitor_id="lib1",
    )
    engine.start_authored_lane()
    try:
        engine.arm(slow)
        engine.arm(library)
        assert _wait_until(lambda: slow.strikes >= 1)
        # While the array monitor is mid-sleep, the TF lane still evaluates:
        # a moved object fires the library monitor synchronously, right now.
        recorder.position = {"x": 0.05, "y": 0.0, "z": 0.0}
        engine.on_tf_tick(recorder)
        lib_fires = [f for f in bus.payloads("monitor_fire")
                     if f["monitor_id"] == "lib1"]
        assert lib_fires and lib_fires[0]["origin"] == "library"
        # And the slow monitor is eventually disarmed with the wall detail.
        assert _wait_until(
            lambda: any(
                p["reason"] == "authored_timeout" and p.get("detail")
                and str(int(ARRAY_WALL_BUDGET_MS)) in p["detail"]
                for p in bus.payloads("monitor_disarmed")
            ),
            timeout=12.0,
        ), bus.payloads("monitor_disarmed")
        assert slow.strikes >= _AUTHORED_STRIKE_LIMIT
    finally:
        engine.stop_authored_lane()


def test_array_lane_fires_through_handle_fire_with_purpose():
    bus = FakeBus()
    engine = MonitorEngine(bus)
    streams = FakeArrayStreams()
    monitor = _monitor(
        DEPTH_CHANGE, streams, engine=engine, rate_hz=2.0,
        params={"change_m": {"value": 0.05, "source": None}},
        purpose="observation",
        threshold_rationale="worth a look when the scene moves this much",
        object_id="u_brown",
    )
    engine.start_authored_lane()
    try:
        engine.arm(monitor)
        armed = bus.payloads("monitor_armed")[0]
        assert armed["purpose"] == "observation"
        assert armed["threshold_rationale"].startswith("worth a look")
        streams.depth_value = 1.42
        assert _wait_until(lambda: bus.payloads("monitor_fire"), timeout=6.0)
        fire = bus.payloads("monitor_fire")[0]
        assert fire["purpose"] == "observation"
        assert fire["origin"] == "authored"
        assert fire["detail"] == "depth changed"
        # A fire wakes wait_for_event like any fire.
        assert engine.wake_event.is_set()
        assert engine.pop_fires()
    finally:
        engine.stop_authored_lane()
