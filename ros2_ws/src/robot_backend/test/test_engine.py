from __future__ import annotations

from types import SimpleNamespace

from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.monitors import EffortEventMonitor


class FakeBus:
    def __init__(self, order=None):
        self.events = []
        self.order = order

    def publish(self, event):
        self.events.append(event)
        if self.order is not None and event.event_type == "monitor_fire":
            self.order.append("publish_fire")


class RecordingWake:
    def __init__(self, order):
        self.order = order
        self.was_set = False

    def set(self):
        self.was_set = True
        self.order.append("wake")


class FakeHalt:
    def __init__(self, order):
        self.order = order
        self.calls = []

    def halt(self, monitor_id, detail, step_id=None):
        self.order.append("halt")
        self.calls.append((monitor_id, detail, step_id))
        return {"ok": True, "detail": "cancelled", "goals_cancelled": 1}


def joint_state(effort):
    return SimpleNamespace(name=["finger_joint"], effort=[effort])


def test_engine_fire_queues_wakes_and_auto_disarms():
    order = []
    wake = RecordingWake(order)
    bus = FakeBus(order)
    halt = FakeHalt(order)
    engine = MonitorEngine(event_bus=bus, halt_client=halt, wake_event=wake)
    monitor = EffortEventMonitor(
        {
            "joint_name": "finger_joint",
            "comparison": "le",
            "threshold": 0.2,
            "sustained_ticks": 1,
        },
        halt_on_fire=True,
        monitor_id="m1",
    )
    engine.arm(monitor)

    engine.on_joint_state(joint_state(0.3))

    fires = engine.pop_fires()
    assert len(fires) == 1
    assert fires[0]["monitor_id"] == "m1"
    assert fires[0]["action_taken"] == "halt"
    assert wake.was_set
    assert engine.snapshot() == []
    # (monitor_id, detail, step_id) -- a monitor-raised halt carries no step
    # label: monitors have no step affiliation, and the halt is located in
    # time by its wall_time_iso like every other monitor event.
    assert halt.calls[0][0] == "m1"
    assert halt.calls[0][2] is None
    assert order.index("halt") < order.index("wake")
    assert [event.event_type for event in bus.events] == [
        "monitor_armed",
        "monitor_fire",
        "monitor_disarmed",
    ]


def test_engine_internal_error_disarms_monitor():
    bus = FakeBus()
    engine = MonitorEngine(event_bus=bus)

    class BadMonitor(EffortEventMonitor):
        def evaluate(self, sample):
            raise RuntimeError("boom")

    monitor = BadMonitor(
        {"joint_name": "finger_joint", "comparison": "gt", "threshold": 0.2, "sustained_ticks": 1},
        monitor_id="bad",
    )
    engine.arm(monitor)
    engine.on_joint_state(joint_state(0.3))

    assert engine.snapshot() == []
    assert bus.events[-1].event_type == "monitor_disarmed"
    assert bus.events[-1].payload["reason"] == "internal_error"
