"""Deterministic WatchDog monitor primitives."""

from __future__ import annotations

import math
import uuid
from typing import Any


def _as_float(value: Any, name: str) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number.") from exc
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite.")
    return out


def _as_positive_float(value: Any, name: str) -> float:
    out = _as_float(value, name)
    if out <= 0.0:
        raise ValueError(f"{name} must be > 0.")
    return out


def _as_positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a positive integer.")
    try:
        out = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive integer.") from exc
    if out <= 0:
        raise ValueError(f"{name} must be a positive integer.")
    return out


def _required_str(params: dict[str, Any], name: str) -> str:
    value = params.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string.")
    return value.strip()


def _xyz(value: Any, name: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping with x, y, z.")
    return {axis: _as_float(value.get(axis), f"{name}.{axis}") for axis in ("x", "y", "z")}


def _pose_position(sample: dict[str, Any]) -> dict[str, float]:
    pose = sample.get("pose", sample)
    position = pose.get("position", pose) if isinstance(pose, dict) else {}
    return _xyz(position, "pose.position")


def _transform_position(transform: Any) -> dict[str, float]:
    if isinstance(transform, dict):
        if "translation" in transform:
            return _xyz(transform["translation"], "translation")
        if "position" in transform:
            return _xyz(transform["position"], "position")
        if all(axis in transform for axis in ("x", "y", "z")):
            return _xyz(transform, "translation")
    tf = getattr(transform, "transform", transform)
    translation = getattr(tf, "translation", None)
    if translation is None:
        raise ValueError("transform must contain a translation.")
    return {"x": float(translation.x), "y": float(translation.y), "z": float(translation.z)}


def _distance(a: dict[str, float], b: dict[str, float]) -> float:
    return math.sqrt(sum((a[axis] - b[axis]) ** 2 for axis in ("x", "y", "z")))


def _compare(value: float, comparison: str, threshold: float) -> bool:
    if comparison == "gt":
        return value > threshold
    if comparison == "ge":
        return value >= threshold
    if comparison == "lt":
        return value < threshold
    if comparison == "le":
        return value <= threshold
    raise ValueError(f"comparison must be one of gt, ge, lt, le; got {comparison!r}.")


class MonitorPrimitive:
    """Base class for deterministic monitor predicates."""

    feed: str = ""
    description: str = ""
    serves: str = ""
    reference_capture: str = "none"
    param_schema: list[dict[str, Any]] = []

    def __init__(
        self,
        monitor_type: str,
        params: dict[str, Any],
        step_id: str,
        clause: dict[str, Any] | None = None,
        halt_on_fire: bool = False,
        monitor_id: str | None = None,
        armed_wall_ns: int | None = None,
    ) -> None:
        if not isinstance(params, dict):
            raise ValueError("params must be a dict.")
        if not isinstance(step_id, str) or not step_id.strip():
            raise ValueError("step_id must be a non-empty string.")
        self.monitor_id = monitor_id or f"mon_{uuid.uuid4().hex[:12]}"
        self.monitor_type = monitor_type
        self.step_id = step_id.strip()
        self.clause = clause
        self.halt_on_fire = bool(halt_on_fire)
        self.armed_wall_ns = int(armed_wall_ns or 0)
        self.params = dict(params)
        self.threshold = float("nan")
        self.comparison = ""
        self.reference_values: dict[str, Any] = {}

    def evaluate(self, sample: Any) -> float | None:
        """Return triggering value when violated, else None."""

        raise NotImplementedError


class RelativeFrameMonitor(MonitorPrimitive):
    """Relative-frame predicate driven by TF; serves retention/alignment clauses."""

    feed = "tf"
    description = (
        "Fires when the straight-line distance between two TF frames departs "
        "from its armed reference by more than max_delta_m. Parameters express "
        "the condition that must hold. The monitor fires when it is violated."
    )
    serves = (
        "Clauses bounding the relative displacement of two frames: grasp "
        "retention between a gripper frame and a held object, insertion "
        "alignment between a part and its mating feature."
    )
    reference_capture = (
        "at_arm_time. The parent->child translation is looked up at the instant "
        "of arming and frozen. The monitor measures departure from that frozen "
        "value, not from any absolute target. Arm only once the relationship "
        "you intend to hold already exists: arming grasp retention before the "
        "grasp closes freezes the pre-grasp separation as the reference and the "
        "monitor will fire on the approach itself. The armed reference is "
        "returned in reference_values.translation; check it. "
        "The reference is a distance between two frames, so any rigidly attached "
        "gripper-side frame serves; a semantically exact grasp point is not "
        "required, and the fixed TCP frame is the only gripper frame published "
        "to TF. Note that the offset between the TCP frame and the actual "
        "grasp centre varies with jaw opening. The armed reference is "
        "therefore valid only while gripper width is held constant. If the "
        "gripper re-grips or changes width after arming, the object moves "
        "relative to the TCP frame and this monitor can fire on a grasp that "
        "was never lost. Disarm and re-arm across any intentional width change. "
        "This is a real property of the RG2 established by calibration, not a "
        "hypothetical."
    )
    param_schema = [
        {"name": "parent_frame", "type": "string", "required": True,
         "notes": "TF frame id. Must exist in the WatchDog TF buffer."},
        {"name": "child_frame", "type": "string", "required": True,
         "notes": "TF frame id."},
        {"name": "max_delta_m", "type": "number", "required": True,
         "units": "meters", "constraint": "> 0",
         "notes": "Permitted departure from the armed reference distance."},
    ]

    def __init__(self, params: dict[str, Any], step_id: str, recorder: Any, **kwargs: Any) -> None:
        super().__init__("relative_frame", params, step_id, **kwargs)
        self.parent_frame = _required_str(params, "parent_frame")
        self.child_frame = _required_str(params, "child_frame")
        self.max_delta_m = _as_positive_float(params.get("max_delta_m"), "max_delta_m")
        self.threshold = self.max_delta_m
        self.comparison = "le"
        try:
            transform = recorder.lookup_transform(self.parent_frame, self.child_frame)
            self._reference = _transform_position(transform)
        except Exception as exc:
            raise ValueError(
                f"reference transform {self.parent_frame}->{self.child_frame} unavailable: {exc}"
            ) from exc
        self.reference_values = {"translation": dict(self._reference)}

    def evaluate(self, sample: Any) -> float | None:
        transform = sample.lookup_transform(self.parent_frame, self.child_frame)
        current = _transform_position(transform)
        delta = _distance(current, self._reference)
        return delta if delta > self.max_delta_m else None


class StationarityMonitor(MonitorPrimitive):
    """Stationarity predicate driven by TF samples; serves scene invariants."""

    feed = "tf"
    description = (
        "Fires when an object's position displaces more than epsilon_m from its "
        "armed reference position. Parameters express the condition that must "
        "hold. The monitor fires when it is violated."
    )
    serves = (
        "Invariants asserting an object is not to be disturbed: untouched-object "
        "invariants, scene invariants over fixtures and neighbouring parts."
    )
    reference_capture = (
        "at_arm_time. The latest buffered pose for object_id becomes the "
        "reference. Arm at step entry, before the executor disturbs the scene; "
        "arming after a disturbance freezes the already-disturbed pose and the "
        "invariant becomes unfalsifiable. Requires at least one buffered pose "
        "sample for the object or arming fails. The armed reference is returned "
        "in reference_values.position."
    )
    param_schema = [
        {"name": "object_id", "type": "string", "required": True,
         "notes": "Must be an object enabled in the model registry."},
        {"name": "epsilon_m", "type": "number", "required": True,
         "units": "meters", "constraint": "> 0",
         "notes": "Permitted displacement from the armed reference position."},
    ]

    def __init__(self, params: dict[str, Any], step_id: str, recorder: Any, **kwargs: Any) -> None:
        super().__init__("stationarity", params, step_id, **kwargs)
        self.object_id = _required_str(params, "object_id")
        self.epsilon_m = _as_positive_float(params.get("epsilon_m"), "epsilon_m")
        self.threshold = self.epsilon_m
        self.comparison = "le"
        latest = recorder.latest_object_pose(self.object_id)
        if latest is None:
            raise ValueError(f"no latest pose sample for object_id {self.object_id!r}.")
        self._reference = _pose_position(latest)
        self.reference_values = {"position": dict(self._reference)}

    def evaluate(self, sample: Any) -> float | None:
        latest = (
            sample.latest_object_pose(self.object_id)
            if hasattr(sample, "latest_object_pose")
            else sample
        )
        if latest is None:
            return None
        current = _pose_position(latest)
        displacement = _distance(current, self._reference)
        return displacement if displacement > self.epsilon_m else None


class RegionMonitor(MonitorPrimitive):
    """Region predicate driven by TF samples; serves pose-corridor clauses."""

    feed = "tf"
    description = (
        "Fires when an object leaves an axis-aligned box (stay_inside) or "
        "enters one (stay_outside). Parameters express the condition that must "
        "hold. The monitor fires when it is violated."
    )
    serves = "Pose corridors, keep-out volumes, workspace bounds."
    reference_capture = (
        "none. The box is absolute and given in the object's registry lookup "
        "frame, so this primitive can be armed at any time with identical "
        "meaning."
    )
    param_schema = [
        {"name": "object_id", "type": "string", "required": True,
         "notes": "Must be an object enabled in the model registry."},
        {"name": "center", "type": "object {x, y, z}", "required": True,
         "units": "meters",
         "notes": "Box center in the object's registry lookup frame."},
        {"name": "half_extents", "type": "object {x, y, z}", "required": True,
         "units": "meters", "constraint": "each > 0",
         "notes": "Half-widths of the box along each axis."},
        {"name": "mode", "type": "enum", "required": True,
         "values": ["stay_inside", "stay_outside"]},
    ]

    def __init__(self, params: dict[str, Any], step_id: str, **kwargs: Any) -> None:
        super().__init__("region", params, step_id, **kwargs)
        self.object_id = _required_str(params, "object_id")
        self.center = _xyz(params.get("center"), "center")
        self.half_extents = _xyz(params.get("half_extents"), "half_extents")
        for axis, value in self.half_extents.items():
            if value <= 0.0:
                raise ValueError(f"half_extents.{axis} must be > 0.")
        self.mode = params.get("mode")
        if self.mode not in {"stay_inside", "stay_outside"}:
            raise ValueError('mode must be "stay_inside" or "stay_outside".')
        self.threshold = 0.0
        self.comparison = "le" if self.mode == "stay_inside" else "gt"

    def evaluate(self, sample: Any) -> float | None:
        latest = (
            sample.latest_object_pose(self.object_id)
            if hasattr(sample, "latest_object_pose")
            else sample
        )
        if latest is None:
            return None
        position = _pose_position(latest)
        clearance = max(
            abs(position[axis] - self.center[axis]) - self.half_extents[axis]
            for axis in ("x", "y", "z")
        )
        if self.mode == "stay_inside":
            return clearance if clearance > 0.0 else None
        return clearance if clearance <= 0.0 else None


class EffortEventMonitor(MonitorPrimitive):
    """Effort-event predicate driven by /joint_states; serves contact/load clauses."""

    feed = "joint_states"
    description = (
        "Watches the absolute effort reported for a named joint against a "
        "condition that must hold. Fires when that condition is violated on "
        "sustained_ticks consecutive samples."
    )
    serves = (
        "Continuous invariants on joint effort, such as clamp-force bounds. Not "
        "for effect clauses that need to hold only at one moment, such as a "
        "minimum plateau effort confirming contact; a continuous monitor for "
        "those would fire from the start of the step, before the moment they "
        "describe has arrived. Evaluate those at step end from evidence."
    )
    reference_capture = (
        "none. The threshold is absolute, so this primitive can be armed at any "
        "time with identical meaning."
    )
    param_schema = [
        {"name": "joint_name", "type": "string", "required": True,
         "notes": "Must appear in the name array of /joint_states."},
        {"name": "comparison", "type": "enum", "required": True,
         "values": ["gt", "ge", "lt", "le"],
         "notes": "The comparison the effort must SATISFY for conformance, "
                  "copied unchanged from the contract clause. The monitor "
                  "fires on violation, not on satisfaction. An invariant of "
                  "'effort <= 4.0' is armed with comparison 'le' and threshold "
                  "4.0 and fires when effort exceeds 4.0."},
        {"name": "threshold", "type": "number", "required": True,
         "units": "effort units as reported on /joint_states",
         "notes": "Compared against the absolute value of the reported effort. "
                  "These are not newtons; use the value from the contract clause "
                  "or registry spec field verbatim and do not convert it."},
        {"name": "sustained_ticks", "type": "integer", "required": False,
         "default": 3, "constraint": "> 0",
         "notes": "Consecutive violating samples required before firing. An "
                  "instrument characterization constant that debounces contact "
                  "chatter, not a specification value. Leave at default unless "
                  "a contract clause states otherwise."},
    ]

    def __init__(self, params: dict[str, Any], step_id: str, **kwargs: Any) -> None:
        super().__init__("effort_event", params, step_id, **kwargs)
        self.joint_name = _required_str(params, "joint_name")
        self.comparison = params.get("comparison")
        if self.comparison not in {"gt", "ge", "lt", "le"}:
            raise ValueError("comparison must be one of gt, ge, lt, le.")
        self.threshold = _as_float(params.get("threshold"), "threshold")
        # Instrument characterization constant: three samples debounces RG2 contact chatter.
        self.sustained_ticks = _as_positive_int(
            params.get("sustained_ticks", 3), "sustained_ticks"
        )
        self._streak = 0

    def evaluate(self, sample: Any) -> float | None:
        names = list(getattr(sample, "name", []))
        if self.joint_name not in names:
            self._streak = 0
            return None
        idx = names.index(self.joint_name)
        efforts = list(getattr(sample, "effort", []))
        if idx >= len(efforts):
            self._streak = 0
            return None
        value = abs(float(efforts[idx]))
        # comparison/threshold express the condition that must HOLD, matching
        # the other three primitives and the meaning of these fields in
        # MonitorFirePayload. Fire when it stops holding.
        if not _compare(value, self.comparison, self.threshold):
            self._streak += 1
            if self._streak >= self.sustained_ticks:
                return value
            return None
        self._streak = 0
        return None


MONITOR_TYPES = {
    "relative_frame": RelativeFrameMonitor,
    "stationarity": StationarityMonitor,
    "region": RegionMonitor,
    "effort_event": EffortEventMonitor,
}


def monitor_definitions() -> list[dict[str, Any]]:
    """Describe every registered monitor primitive."""
    return [
        {
            "monitor_type": name,
            "feed": cls.feed,
            "description": cls.description,
            "serves": cls.serves,
            "reference_capture": cls.reference_capture,
            "params": cls.param_schema,
        }
        for name, cls in MONITOR_TYPES.items()
    ]


__all__ = [
    "MonitorPrimitive",
    "RelativeFrameMonitor",
    "StationarityMonitor",
    "RegionMonitor",
    "EffortEventMonitor",
    "MONITOR_TYPES",
    "monitor_definitions",
]
