"""AuthoredMonitor construction, first call, return contract, and budgets.

All against fakes: an authored monitor touches the recorder and engine only
through its MonitorContext, so a dict-backed recorder and a stub engine are
the whole world it can see -- which is itself one of the properties under
test.
"""

from __future__ import annotations

import hashlib

import pytest

from robot_backend.watchdog.authored import (
    AuthoredBudgetExceeded,
    AuthoredCodeError,
    AuthoredMonitor,
    AuthoredReturnError,
    MonitorContext,
    run_with_budget,
)


class FakeRecorder:
    def __init__(self):
        self.pose = {
            "wall_ns": 1000,
            "object_id": "u_brown",
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        self.joint = {
            "wall_ns": 1000,
            "positions": {"finger_joint": 0.02},
            "efforts": {"finger_joint": 1.5},
        }

    def object_ids(self):
        return ["u_brown"]

    def frame_ids(self):
        return ["tcp"]

    def latest_object_pose(self, object_id):
        return self.pose if object_id == "u_brown" else None

    def latest_frame_pose(self, frame_id):
        return dict(self.pose, frame_id=frame_id) if frame_id == "tcp" else None

    def latest_joint_state(self):
        return self.joint

    def lookup_transform(self, parent, child):
        return {
            "translation": {"x": 0.0, "y": 0.0, "z": 0.09},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

    def get_joint_window(self, start, end, names, max_samples):
        return [
            {"wall_ns": 900, "positions": {"finger_joint": 0.02},
             "efforts": {"finger_joint": 1.4}},
            {"wall_ns": 1000, "positions": {"finger_joint": 0.02},
             "efforts": {"finger_joint": 1.5}},
        ]

    def get_object_pose_window(self, start, end, object_id, max_samples):
        return [self.pose]

    def get_frame_window(self, start, end, frame_id, max_samples):
        return [dict(self.pose, frame_id=frame_id)]


class FakeEngine:
    def __init__(self, held=None):
        self._held = held

    def held_object_id(self):
        return self._held


def _monitor(source, params=None, **kwargs):
    return AuthoredMonitor(
        source=source,
        description="test monitor",
        params=params if params is not None else {},
        recorder=FakeRecorder(),
        engine=FakeEngine(held="u_brown"),
        **kwargs,
    )


PASSIVE = "def check(ctx):\n    return None\n"


# --- construction and the first call ----------------------------------------

def test_first_call_captures_state_and_does_not_fire():
    source = (
        "def check(ctx):\n"
        "    if 'ref' not in ctx.state:\n"
        "        ctx.state['ref'] = ctx.object_pose('u_brown')['pose']['position']\n"
        "        return {'value': 99.0, 'detail': 'first call must not fire'}\n"
        "    return None\n"
    )
    monitor = _monitor(source)
    # The first call ran during construction: the reference is in state and
    # the returned violation was stored, not fired.
    assert monitor.ctx.state["ref"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert monitor.first_call_result == {
        "value": 99.0, "detail": "first call must not fire",
    }
    # The second call sees the captured state and holds.
    assert monitor.evaluate(None) is None


def test_code_sha256_hashes_the_source():
    monitor = _monitor(PASSIVE)
    assert monitor.code_sha256 == hashlib.sha256(PASSIVE.encode()).hexdigest()


def test_monitor_type_feed_and_origin():
    monitor = _monitor(PASSIVE)
    assert monitor.monitor_type == "authored"
    assert monitor.feed == "authored"
    assert monitor.origin == "authored"


def test_validation_failure_carries_stage_validate():
    with pytest.raises(AuthoredCodeError) as exc_info:
        _monitor("import os\ndef check(ctx):\n    return None\n")
    assert exc_info.value.stage == "validate"


def test_runtime_failure_on_first_call_carries_stage_first_call():
    # Exception is not a whitelisted builtin, so naming it fails at runtime,
    # on the first call, not at validation.
    with pytest.raises(AuthoredCodeError) as exc_info:
        _monitor("def check(ctx):\n    return Exception\n")
    assert exc_info.value.stage == "first_call"


def test_malformed_params_carry_stage_params():
    with pytest.raises(AuthoredCodeError) as exc_info:
        _monitor(PASSIVE, params={"eps": 0.02})  # not the {value, source} shape
    assert exc_info.value.stage == "params"


# --- the return contract ------------------------------------------------------

def test_none_means_holds():
    monitor = _monitor(PASSIVE)
    assert monitor.evaluate(None) is None


def test_value_dict_is_a_violation_and_keeps_detail():
    monitor = _monitor(
        "def check(ctx):\n"
        "    return {'value': 0.25, 'detail': 'drifted'}\n"
    )
    assert monitor.evaluate(None) == 0.25
    assert monitor.last_detail == "drifted"


def test_value_dict_without_detail_is_allowed():
    monitor = _monitor("def check(ctx):\n    return {'value': 1.5}\n")
    assert monitor.evaluate(None) == 1.5
    assert monitor.last_detail is None


@pytest.mark.parametrize("returned", ["42", "[1.0]", "{'detail': 'no value'}"])
def test_other_returns_raise_authored_return_error(returned):
    monitor = _monitor(
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        f"    return {returned}\n"
    )
    with pytest.raises(AuthoredReturnError):
        monitor.evaluate(None)


def test_non_finite_value_raises_authored_return_error():
    monitor = _monitor(
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    return {'value': math.inf}\n"
    )
    with pytest.raises(AuthoredReturnError):
        monitor.evaluate(None)


def test_non_str_detail_raises_authored_return_error():
    monitor = _monitor(
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    return {'value': 1.0, 'detail': 7}\n"
    )
    with pytest.raises(AuthoredReturnError):
        monitor.evaluate(None)


# --- reference_values and clause ------------------------------------------------

def test_reference_values_mirror_state_after_first_call():
    source = (
        "def check(ctx):\n"
        "    if 'ref' not in ctx.state:\n"
        "        ctx.state['ref'] = ctx.object_pose('u_brown')['pose']['position']\n"
        "        ctx.state['axes'] = ('x', 'y', 'z')\n"
        "        ctx.state['bad'] = math.inf\n"
        "        ctx.state['count'] = 1\n"
        "    return None\n"
    )
    monitor = _monitor(source)
    assert monitor.reference_values == {
        "ref": {"x": 0.0, "y": 0.0, "z": 0.0},
        "axes": ["x", "y", "z"],   # tuple -> list
        "bad": None,               # non-finite float -> None
        "count": 1,
    }
    # Strictly JSON-serialisable: engine.arm publishes this on monitor_armed.
    import json

    json.dumps(monitor.reference_values, allow_nan=False)


def test_reference_values_replace_non_json_values_with_repr():
    source = (
        "def check(ctx):\n"
        "    ctx.state['names'] = set()\n"
        "    return None\n"
    )
    monitor = _monitor(source)
    assert monitor.reference_values == {"names": repr(set())}


def test_empty_state_yields_empty_reference_values():
    assert _monitor(PASSIVE).reference_values == {}


def test_clause_reaches_the_monitor():
    clause = {"clause_id": "invariants.disturbance_tolerance_m", "object_id": "u_brown"}
    monitor = _monitor(PASSIVE, clause=clause)
    assert monitor.clause == clause


# --- channels_read ------------------------------------------------------------

def test_channels_read_records_methods_and_first_args():
    monitor = _monitor(
        "def check(ctx):\n"
        "    ctx.object_pose('u_brown')\n"
        "    ctx.joint('finger_joint')\n"
        "    ctx.held_object()\n"
        "    return None\n"
    )
    assert monitor.channels_read == [
        "held_object", "joint:finger_joint", "object_pose:u_brown",
    ]


# --- threshold plumbing -------------------------------------------------------

def test_threshold_param_sets_threshold_and_registry_field():
    monitor = _monitor(
        PASSIVE,
        params={
            "eps": {
                "value": 0.02,
                "source": "raw_spec.tolerances.disturbance_tolerance_m",
            },
        },
        threshold_param="eps",
    )
    assert monitor.threshold == 0.02
    assert monitor.comparison == "authored"
    assert monitor.threshold_param == "eps"
    assert monitor.threshold_registry_field == "tolerances.disturbance_tolerance_m"


def test_non_registry_source_yields_no_registry_field():
    monitor = _monitor(
        PASSIVE,
        params={"eps": {"value": 0.02, "source": "task.steps.grasp_width_m"}},
        threshold_param="eps",
    )
    assert monitor.threshold_registry_field is None


def test_missing_source_yields_no_registry_field():
    monitor = _monitor(
        PASSIVE, params={"eps": {"value": 0.02}}, threshold_param="eps"
    )
    assert monitor.threshold_registry_field is None


def test_no_threshold_param_means_threshold_zero():
    monitor = _monitor(PASSIVE)
    assert monitor.threshold == 0.0
    assert monitor.threshold_param is None
    assert monitor.threshold_registry_field is None


def test_threshold_param_must_name_a_param():
    with pytest.raises(AuthoredCodeError) as exc_info:
        _monitor(PASSIVE, params={}, threshold_param="eps")
    assert exc_info.value.stage == "params"


def test_threshold_param_must_be_finite_numeric():
    with pytest.raises(AuthoredCodeError) as exc_info:
        _monitor(
            PASSIVE,
            params={"eps": {"value": "small", "source": None}},
            threshold_param="eps",
        )
    assert exc_info.value.stage == "params"


def test_subject_object_id_is_the_declared_object():
    assert _monitor(PASSIVE, object_id="u_brown").subject_object_id == "u_brown"
    assert _monitor(PASSIVE).subject_object_id is None


# --- budgets ------------------------------------------------------------------

def test_budget_exceeded_on_a_long_for_loop():
    monitor = _monitor(
        "def check(ctx):\n"
        "    if 'armed' not in ctx.state:\n"
        "        ctx.state['armed'] = True\n"
        "        return None\n"
        "    total = 0\n"
        "    for i in range(100000):\n"
        "        total = total + i\n"
        "    return None\n"
    )
    with pytest.raises(AuthoredBudgetExceeded):
        monitor.evaluate(None)


def test_run_with_budget_restores_previous_tracer():
    import sys

    previous = sys.gettrace()
    result, elapsed = run_with_budget(lambda arg: arg + 1, 1, max_lines=100)
    assert result == 2
    assert elapsed >= 0.0
    assert sys.gettrace() is previous


def test_run_with_budget_restores_tracer_after_overrun():
    import sys

    previous = sys.gettrace()

    def runaway(_arg):
        total = 0
        for i in range(100000):
            total += i
        return total

    with pytest.raises(AuthoredBudgetExceeded):
        run_with_budget(runaway, None, max_lines=50)
    assert sys.gettrace() is previous


# --- the sandbox has no way out -----------------------------------------------

def test_context_is_the_whole_world():
    """Names outside the whitelist and ctx internals are unreachable."""
    # The validator stops underscore attributes at arm time...
    with pytest.raises(AuthoredCodeError):
        _monitor("def check(ctx):\n    return ctx._recorder\n")
    # ...and the builtins whitelist stops everything it does not name.
    for name in ("open", "getattr", "type", "__import__"):
        with pytest.raises(AuthoredCodeError):
            _monitor(f"def check(ctx):\n    return {name}\n")


def test_context_returns_fresh_copies():
    recorder = FakeRecorder()
    ctx = MonitorContext(recorder, FakeEngine(), params={}, state={})
    pose = ctx.object_pose("u_brown")
    pose["pose"]["position"]["x"] = 123.0
    assert recorder.pose["pose"]["position"]["x"] == 0.0


def test_context_has_no_step_events_or_registry_surface():
    ctx = MonitorContext(FakeRecorder(), FakeEngine(), params={}, state={})
    public = [name for name in dir(ctx) if not name.startswith("_")]
    assert sorted(public) == [
        "channels_read", "dist", "frame_ids", "frame_pose", "frame_pose_window",
        "held_object", "joint", "joint_names", "joint_window", "now_wall_ns",
        "object_ids", "object_pose", "object_pose_window", "params", "state",
        "transform",
    ]
