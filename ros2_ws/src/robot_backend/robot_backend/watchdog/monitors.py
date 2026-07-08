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
        if _compare(value, self.comparison, self.threshold):
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
