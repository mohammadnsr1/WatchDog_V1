"""The authored lane: fires travel _handle_fire, failures disarm, library
monitors are untouched by an authored monitor that raises or loops.

These tests run the real lane thread against a fake event bus, synchronising
on published events (or a call counter in ctx.state) rather than sleeps of
hope: the lane wakes on the same on_tf_tick/on_joint_state entry points the
recorder drives in production.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from robot_backend.watchdog.authored import AuthoredMonitor
from robot_backend.watchdog.engine import (
    _AUTHORED_STRIKE_LIMIT,
    MonitorEngine,
)
from robot_backend.watchdog.monitors import StationarityMonitor


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


class FakeRecorder:
    """Mutable latest-sample world for both library and authored monitors."""

    def __init__(self):
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}

    def object_ids(self):
        return ["u_brown"]

    def frame_ids(self):
        return []

    def latest_object_pose(self, object_id):
        if object_id != "u_brown":
            return None
        return {
            "wall_ns": 0,
            "object_id": "u_brown",
            "pose": {
                "position": dict(self.position),
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

    def latest_frame_pose(self, frame_id):
        return None

    def latest_joint_state(self):
        return None

    def lookup_transform(self, parent, child):
        raise RuntimeError("no TF in this fake")

    def get_joint_window(self, start, end, names, max_samples):
        return []

    def get_object_pose_window(self, start, end, object_id, max_samples):
        return []

    def get_frame_window(self, start, end, frame_id, max_samples):
        return []


def _wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


@pytest.fixture
def world():
    bus = FakeBus()
    engine = MonitorEngine(event_bus=bus)
    recorder = FakeRecorder()
    engine.attach_recorder(recorder)
    engine.start_authored_lane()
    yield SimpleNamespace(bus=bus, engine=engine, recorder=recorder)
    engine.stop_authored_lane()


def _authored(world, source, monitor_id="auth1", **kwargs):
    return AuthoredMonitor(
        source=source,
        description="lane test monitor",
        params=kwargs.pop("params", {}),
        recorder=world.recorder,
        engine=world.engine,
        monitor_id=monitor_id,
        **kwargs,
    )


def _wake(world):
    """One production-shaped wake: the TF tick entry point."""
    world.engine.on_tf_tick(world.recorder)


COUNTING_DRIFT = (
    "def check(ctx):\n"
    "    ctx.state['calls'] = ctx.state.get('calls', 0) + 1\n"
    "    if 'ref' not in ctx.state:\n"
    "        ctx.state['ref'] = ctx.object_pose('u_brown')['pose']['position']\n"
    "        return None\n"
    "    d = ctx.dist(ctx.object_pose('u_brown')['pose']['position'], ctx.state['ref'])\n"
    "    if d > 0.02:\n"
    "        return {'value': d, 'detail': 'drifted'}\n"
    "    return None\n"
)


def test_authored_fire_publishes_through_handle_fire(world):
    monitor = _authored(world, COUNTING_DRIFT, object_id="u_brown")
    world.engine.arm(monitor)

    # Holds while nothing moved.
    _wake(world)
    assert _wait_until(lambda: monitor.ctx.state.get("calls", 0) >= 2)
    assert not world.bus.payloads("monitor_fire")

    # Move the object beyond the authored epsilon and wake once more.
    world.recorder.position = {"x": 0.05, "y": 0.0, "z": 0.0}
    _wake(world)
    assert _wait_until(lambda: world.bus.payloads("monitor_fire"))

    fire = world.bus.payloads("monitor_fire")[0]
    assert fire["monitor_id"] == "auth1"
    assert fire["monitor_type"] == "authored"
    assert fire["origin"] == "authored"
    assert fire["detail"] == "drifted"
    assert fire["code_sha256"] == monitor.code_sha256
    assert fire["triggering_value"] == pytest.approx(0.05)
    # Fired means disarmed, exactly as a library fire is.
    disarms = world.bus.payloads("monitor_disarmed")
    assert disarms and disarms[-1]["reason"] == "fired"
    assert world.engine.snapshot() == []
    # The fire also reached the queue wait_for_event drains.
    fires = world.engine.pop_fires()
    assert fires and fires[0]["monitor_id"] == "auth1"


def test_armed_event_carries_authored_fields(world):
    monitor = _authored(world, COUNTING_DRIFT, object_id="u_brown")
    world.engine.arm(monitor)
    armed = world.bus.payloads("monitor_armed")[0]
    assert armed["origin"] == "authored"
    assert armed["code_sha256"] == monitor.code_sha256
    assert armed["description"] == "lane test monitor"
    assert "object_pose:u_brown" in armed["channels_read"]


def test_budget_overruns_disarm_after_strike_limit(world):
    source = (
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    total = 0\n"
        "    for i in range(100000):\n"
        "        total = total + i\n"
        "    return None\n"
    )
    monitor = _authored(world, source)
    world.engine.arm(monitor)
    for expected in range(1, _AUTHORED_STRIKE_LIMIT + 1):
        _wake(world)
        # Let the lane drain this wake before issuing the next, so each wake
        # is exactly one evaluation and one strike.
        assert _wait_until(lambda: monitor.strikes >= expected)
    assert _wait_until(lambda: world.bus.payloads("monitor_disarmed"))
    disarm = world.bus.payloads("monitor_disarmed")[-1]
    assert disarm["reason"] == "authored_timeout"
    assert not world.bus.payloads("monitor_fire")


def test_bad_return_disarms(world):
    source = (
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    return 42\n"
    )
    monitor = _authored(world, source)
    world.engine.arm(monitor)
    _wake(world)
    assert _wait_until(lambda: world.bus.payloads("monitor_disarmed"))
    assert world.bus.payloads("monitor_disarmed")[-1]["reason"] == "authored_bad_return"
    assert not world.bus.payloads("monitor_fire")
    assert monitor.first_call_result is None


def test_runtime_error_disarms(world):
    source = (
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    return ctx.params['missing']\n"
    )
    monitor = _authored(world, source)
    world.engine.arm(monitor)
    _wake(world)
    assert _wait_until(lambda: world.bus.payloads("monitor_disarmed"))
    assert world.bus.payloads("monitor_disarmed")[-1]["reason"] == "authored_error"


def test_library_monitors_are_untouched_by_a_broken_authored_monitor(world):
    # An authored monitor that loops past its budget on every call...
    runaway = (
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    total = 0\n"
        "    for i in range(100000):\n"
        "        total = total + i\n"
        "    return None\n"
    )
    world.engine.arm(_authored(world, runaway))
    # ...beside a library stationarity monitor over the same fake recorder.
    library = StationarityMonitor(
        {"object_id": "u_brown", "epsilon_m": 0.02},
        recorder=world.recorder,
        monitor_id="lib1",
    )
    world.engine.arm(library)

    # The library monitor is evaluated synchronously inside on_tf_tick, on
    # this thread, whatever the authored lane is doing.
    world.recorder.position = {"x": 0.05, "y": 0.0, "z": 0.0}
    world.engine.on_tf_tick(world.recorder)

    fires = world.bus.payloads("monitor_fire")
    assert [f["monitor_id"] for f in fires] == ["lib1"]
    assert fires[0]["origin"] == "library"
    assert fires[0]["triggering_value"] == pytest.approx(0.05)


def test_snapshot_carries_authored_identity(world):
    monitor = _authored(world, COUNTING_DRIFT, object_id="u_brown")
    world.engine.arm(monitor)
    snap = world.engine.snapshot()
    assert snap[0]["origin"] == "authored"
    assert snap[0]["code_sha256"] == monitor.code_sha256
    assert snap[0]["description"] == "lane test monitor"


def test_stop_authored_lane_joins_the_thread(world):
    thread = world.engine._authored_thread
    assert thread is not None and thread.is_alive()
    world.engine.stop_authored_lane()
    assert not thread.is_alive()
