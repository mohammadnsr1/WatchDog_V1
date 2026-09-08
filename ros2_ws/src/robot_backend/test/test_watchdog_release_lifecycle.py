"""Slip-monitor lifecycle around release, and structural fire attribution.

Covers two mechanical fixes:
  * a relative_frame slip monitor is torn down when its part is released, so
    the opening jaws cannot register as a slip (and the teardown is on the
    record with its own reason);
  * every fire carries a structurally-resolved object_id, independent of what
    the agent typed into the clause.
"""

import threading

from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.monitors import MonitorPrimitive


class _Bus:
    def __init__(self):
        self.events = []

    def publish(self, event):
        self.events.append(event)

    @staticmethod
    def _field(payload, name):
        return payload[name] if isinstance(payload, dict) else getattr(payload, name)

    def disarms(self):
        return [
            e.payload for e in self.events if e.event_type == "monitor_disarmed"
        ]

    def disarm_reasons(self):
        return [self._field(p, "reason") for p in self.disarms()]


class _Slip(MonitorPrimitive):
    """Stand-in for RelativeFrameMonitor: subject is the held part."""

    feed = "tf"

    def __init__(self, monitor_id, child_frame, clause=None):
        super().__init__(
            "relative_frame",
            {"parent_frame": "tcp", "child_frame": child_frame},
            clause=clause,
            monitor_id=monitor_id,
        )
        self.child_frame = child_frame
        self.threshold = 0.01
        self.comparison = "le"
        self._fire = None

    @property
    def subject_object_id(self):
        return self.child_frame

    def evaluate(self, sample):
        return self._fire


class _Effort(MonitorPrimitive):
    """Stand-in for EffortEventMonitor: joint-scoped, no intrinsic object."""

    feed = "joint_states"

    def __init__(self, monitor_id, clause=None):
        super().__init__(
            "effort_event", {"joint_name": "finger_joint"},
            clause=clause, monitor_id=monitor_id,
        )
        self.threshold = 16.0
        self.comparison = "le"
        self._fire = None

    def evaluate(self, sample):
        return self._fire


class _Stationarity(MonitorPrimitive):
    feed = "tf"

    def __init__(self, monitor_id, object_id):
        super().__init__(
            "stationarity", {"object_id": object_id, "epsilon_m": 0.005},
            monitor_id=monitor_id,
        )
        self.object_id = object_id
        self.threshold = 0.005
        self.comparison = "le"
        self._fire = None

    @property
    def subject_object_id(self):
        return self.object_id

    def evaluate(self, sample):
        return self._fire


def _engine(bus=None):
    return MonitorEngine(event_bus=bus or _Bus(), wake_event=threading.Event())


def test_release_disarms_only_that_parts_slip_monitor():
    bus = _Bus()
    eng = _engine(bus)
    eng.arm(_Slip("slip_green", "u_green"))
    eng.arm(_Slip("slip_brown", "inverted_u_brown"))
    eng.arm(_Stationarity("stat_blue", "hex_blue"))

    eng.on_skill_event("skill_start", "release_skill", {"object_id": "u_green"})

    still_armed = {m["monitor_id"] for m in eng.snapshot()}
    assert still_armed == {"slip_brown", "stat_blue"}
    assert "auto_disarm_release" in bus.disarm_reasons()


def test_auto_disarm_is_on_the_record_with_its_own_reason():
    bus = _Bus()
    eng = _engine(bus)
    eng.arm(_Slip("slip_green", "u_green"))
    eng.on_skill_event("skill_start", "release_skill", {"object_id": "u_green"})

    disarms = bus.disarms()
    assert len(disarms) == 1
    assert bus._field(disarms[0], "monitor_id") == "slip_green"
    # Distinguishable from an agent-initiated disarm and from "never armed".
    assert bus._field(disarms[0], "reason") == "auto_disarm_release"


def test_slip_before_release_still_fires():
    """A genuine mid-span slip is untouched: the monitor is still armed."""
    eng = _engine()
    slip = _Slip("slip_brown", "inverted_u_brown")
    eng.arm(slip)
    slip._fire = 0.0105          # the real brown insertion slip, mid-descent
    eng.on_tf_tick(object())
    fires = eng.pop_fires()
    assert len(fires) == 1
    assert fires[0]["object_id"] == "inverted_u_brown"


def test_slip_at_release_cannot_fire():
    eng = _engine()
    slip = _Slip("slip_green", "u_green")
    eng.arm(slip)
    eng.on_skill_event("skill_start", "release_skill", {"object_id": "u_green"})
    slip._fire = 0.021           # jaws opening, would have been a phantom
    eng.on_tf_tick(object())
    assert eng.pop_fires() == []


def test_setdown_disarms_and_repick_rearms():
    """Reorient set-down is a release; the re-pick arms a fresh monitor."""
    bus = _Bus()
    eng = _engine(bus)
    eng.arm(_Slip("slip_1", "u_green"))
    eng.on_skill_event("skill_start", "release_skill", {"object_id": "u_green"})
    assert eng.snapshot() == []

    eng.on_skill_event("skill_start", "grasp_skill", {"object_id": "u_green"})
    eng.on_skill_event("skill_end", "grasp_skill", None, ok=True)
    slip2 = _Slip("slip_2", "u_green")
    eng.arm(slip2)
    slip2._fire = 0.02
    eng.on_tf_tick(object())
    fires = eng.pop_fires()
    assert len(fires) == 1 and fires[0]["object_id"] == "u_green"


def test_effort_fire_attributes_to_held_part():
    eng = _engine()
    eng.on_skill_event("skill_start", "grasp_skill", {"object_id": "u_green"})
    eng.on_skill_event("skill_end", "grasp_skill", None, ok=True)
    eff = _Effort("eff_1")
    eng.arm(eff)
    eff._fire = 140.13
    eng.on_joint_state(object())
    fires = eng.pop_fires()
    assert fires[0]["object_id"] == "u_green"


def test_effort_fire_during_close_attributes_before_confirmation():
    """Clamp bound exceeded mid-close still names the part being closed on."""
    eng = _engine()
    eng.on_skill_event("skill_start", "grasp_skill", {"object_id": "hex_red"})
    eff = _Effort("eff_1")
    eng.arm(eff)
    eff._fire = 99.0
    eng.on_joint_state(object())
    assert eng.pop_fires()[0]["object_id"] == "hex_red"


def test_effort_fire_with_empty_gripper_is_none():
    eng = _engine()
    eng.on_skill_event("skill_start", "grasp_skill", {"object_id": "u_green"})
    eng.on_skill_event("skill_end", "grasp_skill", None, ok=True)
    eng.on_skill_event("skill_start", "release_skill", {"object_id": "u_green"})
    eff = _Effort("eff_1")
    eng.arm(eff)
    eff._fire = 5.0
    eng.on_joint_state(object())
    assert eng.pop_fires()[0]["object_id"] is None


def test_failed_grasp_leaves_nothing_held():
    eng = _engine()
    eng.on_skill_event("skill_start", "grasp_skill", {"object_id": "u_green"})
    eng.on_skill_event("skill_end", "grasp_skill", None, ok=False)
    assert eng.held_object_id() is None


def test_clause_object_id_backfilled_but_agent_value_wins():
    eng = _engine()
    slip = _Slip("s1", "u_green", clause={"clause_id": "x.y", "object_id": None})
    eng.arm(slip)
    slip._fire = 0.02
    eng.on_tf_tick(object())
    assert eng.pop_fires()[0]["clause"]["object_id"] == "u_green"

    eng2 = _engine()
    slip2 = _Slip("s2", "u_green", clause={"clause_id": "x.y", "object_id": "agent_said"})
    eng2.arm(slip2)
    slip2._fire = 0.02
    eng2.on_tf_tick(object())
    assert eng2.pop_fires()[0]["clause"]["object_id"] == "agent_said"
