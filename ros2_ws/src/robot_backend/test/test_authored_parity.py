"""Authored reimplementations of the library primitives behave identically.

For stationarity, relative_frame and effort_event, an authored check() written
against the MonitorContext API is driven through the real authored lane by the
same synthetic sample sequence that drives the library primitive through
_evaluate_feed -- one lane wake per sample -- and both must fire on the same
sample with the same triggering value. This is the ground truth for S2/S3
comparisons: any behavioural gap between the two lanes would contaminate every
scenario comparison built on top of them.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from robot_backend.watchdog.authored import AuthoredMonitor
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.monitors import (
    EffortEventMonitor,
    RelativeFrameMonitor,
    StationarityMonitor,
)


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
    """Mutable latest-sample world shared by both lanes."""

    def __init__(self):
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.translation = {"x": 0.0, "y": 0.0, "z": 0.09}
        self.effort = 0.0

    def object_ids(self):
        return ["u_brown"]

    def frame_ids(self):
        return ["tcp"]

    def latest_object_pose(self, object_id):
        return {
            "wall_ns": 0,
            "object_id": object_id,
            "pose": {
                "position": dict(self.position),
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

    def latest_frame_pose(self, frame_id):
        return None

    def latest_joint_state(self):
        return {
            "wall_ns": 0,
            "positions": {"finger_joint": 0.02},
            "efforts": {"finger_joint": self.effort},
        }

    def lookup_transform(self, parent, child):
        return {
            "translation": dict(self.translation),
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

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


def _fires_by_id(world):
    return {f["monitor_id"]: f for f in world.bus.payloads("monitor_fire")}


def _drive(world, monitor, samples, apply_sample, wake):
    """Feed samples one at a time, one lane wake per sample, synchronised.

    apply_sample(sample) mutates the fake recorder; wake(sample) enters the
    engine through the same production entry point the recorder uses, which
    evaluates the library monitor synchronously and wakes the lane. After
    each wake, the loop blocks until the authored monitor has consumed it:
    the authored sources below advance ctx.state["calls"] as their final
    statement, so a bumped counter means the sample's reads are all done and
    the next mutation cannot race them. A check that decided to fire sets
    ctx.state["fired"] first, and the loop then waits for the fire event to
    be published before attributing it to this sample. Returns
    {monitor_id: sample_index_at_fire}.
    """
    fired_at: dict[str, int] = {}
    for index, sample in enumerate(samples):
        apply_sample(sample)
        calls_before = monitor.ctx.state.get("calls", 0)
        authored_done = monitor.monitor_id in _fires_by_id(world)
        wake(sample)
        if not authored_done:
            assert _wait_until(
                lambda: monitor.ctx.state.get("calls", 0) > calls_before
            )
            if monitor.ctx.state.get("fired"):
                assert _wait_until(
                    lambda: monitor.monitor_id in _fires_by_id(world)
                )
        for monitor_id in _fires_by_id(world):
            fired_at.setdefault(monitor_id, index)
        if len(fired_at) >= 2:
            break
    return fired_at


AUTHORED_STATIONARITY = (
    "def check(ctx):\n"
    "    pos = ctx.object_pose('u_brown')['pose']['position']\n"
    "    result = None\n"
    "    if 'ref' not in ctx.state:\n"
    "        ctx.state['ref'] = pos\n"
    "    else:\n"
    "        d = ctx.dist(pos, ctx.state['ref'])\n"
    "        if d > ctx.value('epsilon_m'):\n"
    "            result = {'value': d}\n"
    "    if result is not None:\n"
    "        ctx.state['fired'] = True\n"
    "    ctx.state['calls'] = ctx.state.get('calls', 0) + 1\n"
    "    return result\n"
)

AUTHORED_RELATIVE_FRAME = (
    "def check(ctx):\n"
    "    t = ctx.transform('tcp', 'u_brown')['translation']\n"
    "    result = None\n"
    "    if 'ref' not in ctx.state:\n"
    "        ctx.state['ref'] = t\n"
    "    else:\n"
    "        d = ctx.dist(t, ctx.state['ref'])\n"
    "        if d > ctx.value('max_delta_m'):\n"
    "            result = {'value': d}\n"
    "    if result is not None:\n"
    "        ctx.state['fired'] = True\n"
    "    ctx.state['calls'] = ctx.state.get('calls', 0) + 1\n"
    "    return result\n"
)

AUTHORED_EFFORT = (
    "def check(ctx):\n"
    "    sample = ctx.joint('finger_joint')\n"
    "    result = None\n"
    "    if sample is None:\n"
    "        ctx.state['streak'] = 0\n"
    "    else:\n"
    "        value = abs(sample['effort'])\n"
    "        if value > ctx.value('threshold'):\n"
    "            ctx.state['streak'] = ctx.state.get('streak', 0) + 1\n"
    "            if ctx.state['streak'] >= 3:\n"
    "                result = {'value': value}\n"
    "        else:\n"
    "            ctx.state['streak'] = 0\n"
    "    if result is not None:\n"
    "        ctx.state['fired'] = True\n"
    "    ctx.state['calls'] = ctx.state.get('calls', 0) + 1\n"
    "    return result\n"
)


def _authored(world, source, params, **kwargs):
    return AuthoredMonitor(
        source=source,
        description="parity monitor",
        params=params,
        recorder=world.recorder,
        engine=world.engine,
        monitor_id="authored",
        **kwargs,
    )


def test_stationarity_parity(world):
    epsilon = 0.02
    library = StationarityMonitor(
        {"object_id": "u_brown", "epsilon_m": epsilon},
        recorder=world.recorder,
        monitor_id="library",
    )
    authored = _authored(
        world,
        AUTHORED_STATIONARITY,
        params={"epsilon_m": {"value": epsilon, "source": None}},
        object_id="u_brown",
    )
    world.engine.arm(library)
    world.engine.arm(authored)

    drift = [0.0, 0.005, 0.010, 0.015, 0.019, 0.021, 0.05]
    fired_at = _drive(
        world,
        authored,
        drift,
        apply_sample=lambda x: setattr(
            world.recorder, "position", {"x": x, "y": 0.0, "z": 0.0}
        ),
        wake=lambda sample: world.engine.on_tf_tick(world.recorder),
    )

    fires = _fires_by_id(world)
    assert set(fires) == {"library", "authored"}
    assert fired_at["library"] == fired_at["authored"] == drift.index(0.021)
    assert fires["authored"]["triggering_value"] == pytest.approx(
        fires["library"]["triggering_value"]
    )
    assert fires["library"]["triggering_value"] == pytest.approx(0.021)


def test_relative_frame_parity(world):
    max_delta = 0.02
    library = RelativeFrameMonitor(
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": max_delta},
        recorder=world.recorder,
        monitor_id="library",
    )
    authored = _authored(
        world,
        AUTHORED_RELATIVE_FRAME,
        params={"max_delta_m": {"value": max_delta, "source": None}},
        object_id="u_brown",
    )
    world.engine.arm(library)
    world.engine.arm(authored)

    slips = [0.0, 0.004, 0.008, 0.012, 0.016, 0.020, 0.025, 0.06]
    fired_at = _drive(
        world,
        authored,
        slips,
        apply_sample=lambda s: setattr(
            world.recorder, "translation", {"x": 0.0, "y": s, "z": 0.09}
        ),
        wake=lambda sample: world.engine.on_tf_tick(world.recorder),
    )

    fires = _fires_by_id(world)
    assert set(fires) == {"library", "authored"}
    assert fired_at["library"] == fired_at["authored"] == slips.index(0.025)
    assert fires["authored"]["triggering_value"] == pytest.approx(
        fires["library"]["triggering_value"]
    )


def test_effort_event_parity_with_streak_in_state(world):
    threshold = 4.0
    library = EffortEventMonitor(
        {
            "joint_name": "finger_joint",
            "comparison": "le",
            "threshold": threshold,
            "sustained_ticks": 3,
        },
        monitor_id="library",
    )
    authored = _authored(
        world,
        AUTHORED_EFFORT,
        params={"threshold": {"value": threshold, "source": None}},
    )
    world.engine.arm(library)
    world.engine.arm(authored)

    # Two violating samples, a reset, then three in a row: only the second
    # streak completes, on the last sample.
    efforts = [1.0, 4.5, 4.6, 2.0, 4.7, 4.8, 4.9]

    def apply_sample(effort):
        world.recorder.effort = effort

    def wake(effort):
        world.engine.on_joint_state(
            SimpleNamespace(name=["finger_joint"], effort=[effort])
        )

    fired_at = _drive(world, authored, efforts, apply_sample, wake)

    fires = _fires_by_id(world)
    assert set(fires) == {"library", "authored"}
    assert fired_at["library"] == fired_at["authored"] == efforts.index(4.9)
    assert fires["authored"]["triggering_value"] == pytest.approx(4.9)
    assert fires["library"]["triggering_value"] == pytest.approx(4.9)
