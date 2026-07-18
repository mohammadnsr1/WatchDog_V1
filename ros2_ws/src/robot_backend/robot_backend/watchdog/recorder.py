"""Always-on evidence recorder for the WatchDog node."""

from __future__ import annotations

import collections
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from robot_backend.events.schema import EventEnvelope

from .timebase import wall_ns

JOINT_STATES_TOPIC = "/joint_states"
EVENTS_TOPIC = "/watchdog/events"
TF_SAMPLE_HZ = 20.0
MAX_JOINT_SAMPLES = 60 * 60 * 10  # 60 Hz * 60 sec/min * 10 min.
MAX_TF_SAMPLES = int(TF_SAMPLE_HZ * 60 * 10)  # 20 Hz * 60 sec/min * 10 min.

_DEFAULT_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "model_registry.yaml"
)


def _load_registry(path: Path | None = None) -> dict[str, Any]:
    if path is not None:
        resolved = path
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            resolved = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
        except Exception:
            resolved = _DEFAULT_REGISTRY_PATH
    with resolved.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError("model_registry.yaml must be a top-level mapping.")
    return data


def _enabled_object_frames(registry: dict[str, Any]) -> dict[str, dict[str, str]]:
    frames: dict[str, dict[str, str]] = {}
    objects = registry.get("objects", {})
    if not isinstance(objects, dict):
        return frames
    for object_id, entry in objects.items():
        if not isinstance(entry, dict) or not entry.get("enabled", True):
            continue
        tf_frame = entry.get("tf_frame")
        lookup_frame = entry.get("lookup_frame")
        if (
            isinstance(tf_frame, str)
            and tf_frame
            and isinstance(lookup_frame, str)
            and lookup_frame
        ):
            frames[str(object_id)] = {"tf_frame": tf_frame, "lookup_frame": lookup_frame}
    return frames


def _parse_wall_time_iso(value: Any) -> int | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return int(parsed.timestamp() * 1_000_000_000)
    except Exception:
        return None


def _uniform_downsample(items: list[Any], max_samples: int = 200) -> list[Any]:
    if max_samples <= 0:
        return []
    if len(items) <= max_samples:
        return items
    if max_samples == 1:
        return [items[-1]]
    last = len(items) - 1
    indices = sorted({round(i * last / (max_samples - 1)) for i in range(max_samples)})
    return [items[i] for i in indices]


def _stamp(sample: Any) -> int:
    if isinstance(sample, dict):
        return int(sample.get("wall_ns", sample.get("wall_ns_received", 0)))
    return int(sample[0])


class EvidenceRecorder:
    """Rolling recorder for joint states, object TF poses, and executor events."""

    def __init__(
        self,
        node: Any | None,
        registry: dict[str, Any] | None = None,
        registry_path: Path | None = None,
        engine: Any | None = None,
        lock: threading.Lock | None = None,
        wake_event: threading.Event | None = None,
    ) -> None:
        self.node = node
        self.engine = engine
        self.lock = lock or threading.Lock()
        self.wake_event = wake_event or threading.Event()
        self.registry = registry if registry is not None else _load_registry(registry_path)
        self.object_frames = _enabled_object_frames(self.registry)
        self.joint_samples: collections.deque[Any] = collections.deque(maxlen=MAX_JOINT_SAMPLES)
        self.object_pose_samples: dict[str, collections.deque[Any]] = {
            oid: collections.deque(maxlen=MAX_TF_SAMPLES) for oid in self.object_frames
        }
        self.events: collections.deque[dict[str, Any]] = collections.deque(maxlen=5000)
        self._wait_events: collections.deque[dict[str, Any]] = collections.deque(maxlen=1000)
        self.tf_staleness_counters: dict[str, int] = {oid: 0 for oid in self.object_frames}
        self.current_step_id: str | None = None
        self.tf_buffer: Any | None = None
        self.tf_listener: Any | None = None
        self._subs: list[Any] = []
        self._timer: Any | None = None

        if node is not None:
            self._attach_ros(node)

    def _attach_ros(self, node: Any) -> None:
        try:
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            import tf2_ros
        except Exception as exc:
            raise RuntimeError(f"WatchDog recorder ROS imports failed: {exc}") from exc

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self._subs.append(
            node.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 100)
        )
        self._subs.append(
            node.create_subscription(String, EVENTS_TOPIC, self._on_event_msg, 100)
        )
        self._timer = node.create_timer(1.0 / TF_SAMPLE_HZ, self._on_tf_timer)

    def _log_warn(self, message: str) -> None:
        logger = (
            getattr(self.node, "get_logger", lambda: None)()
            if self.node is not None
            else None
        )
        if logger is not None:
            try:
                logger.warning(message)
            except Exception:
                pass

    def _on_joint_state(self, msg: Any) -> None:
        try:
            stamp = wall_ns()
            names = list(getattr(msg, "name", []))
            positions = list(getattr(msg, "position", []))
            efforts = list(getattr(msg, "effort", []))
            position_by_name = {
                name: float(positions[idx])
                for idx, name in enumerate(names)
                if idx < len(positions)
            }
            effort_by_name = {
                name: float(efforts[idx]) for idx, name in enumerate(names) if idx < len(efforts)
            }
            with self.lock:
                self.joint_samples.append((stamp, position_by_name, effort_by_name))
            if self.engine is not None:
                self.engine.on_joint_state(msg)
        except Exception as exc:
            self._log_warn(f"WatchDog joint-state callback failed: {exc}")

    def _on_tf_timer(self) -> None:
        try:
            for object_id in list(self.object_frames):
                frame = self.object_frames[object_id]
                try:
                    transform = self.lookup_transform(frame["lookup_frame"], frame["tf_frame"])
                    pose = self._transform_to_pose_tuple(wall_ns(), object_id, transform)
                    with self.lock:
                        self.object_pose_samples.setdefault(
                            object_id, collections.deque(maxlen=MAX_TF_SAMPLES)
                        ).append(pose)
                except Exception:
                    with self.lock:
                        self.tf_staleness_counters[object_id] = (
                            self.tf_staleness_counters.get(object_id, 0) + 1
                        )
            if self.engine is not None:
                self.engine.on_tf_tick(self)
        except Exception as exc:
            self._log_warn(f"WatchDog TF timer failed: {exc}")

    def _on_event_msg(self, msg: Any) -> None:
        try:
            received = wall_ns()
            envelope = EventEnvelope.model_validate_json(msg.data)
            envelope_dict = envelope.model_dump(mode="json")
            record = {
                "wall_ns_received": received,
                "envelope_wall_ns": _parse_wall_time_iso(envelope.wall_time_iso),
                "event_type": envelope.event_type,
                "payload": envelope.payload,
                "envelope": envelope_dict,
            }
            step_id = (
                envelope.payload.get("step_id")
                if isinstance(envelope.payload, dict)
                else None
            )
            with self.lock:
                self.events.append(record)
                self._wait_events.append(record)
                if envelope.event_type == "begin_step" and isinstance(step_id, str):
                    self.current_step_id = step_id
                elif envelope.event_type == "end_step":
                    self.current_step_id = None
            self.wake_event.set()
        except Exception as exc:
            self._log_warn(f"WatchDog event-tail callback failed: {exc}")

    def _transform_to_pose_tuple(
        self, stamp: int, object_id: str, transform: Any
    ) -> tuple[Any, ...]:
        tf = getattr(transform, "transform", transform)
        translation = getattr(tf, "translation")
        rotation = getattr(tf, "rotation")
        return (
            stamp,
            object_id,
            float(translation.x),
            float(translation.y),
            float(translation.z),
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )

    def lookup_transform(self, parent_frame: str, child_frame: str) -> Any:
        if self.tf_buffer is None:
            raise RuntimeError("TF buffer is not attached.")
        try:
            import rclpy
            return self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
        except Exception:
            return self.tf_buffer.lookup_transform(parent_frame, child_frame, None)

    def get_joint_window(
        self,
        start_wall_ns: int,
        end_wall_ns: int,
        joint_names: list[str] | None = None,
        max_samples: int = 200,
    ) -> list[dict[str, Any]]:
        wanted = set(joint_names or [])
        with self.lock:
            samples = [s for s in self.joint_samples if start_wall_ns <= int(s[0]) <= end_wall_ns]
        rows = []
        for stamp, positions, efforts in _uniform_downsample(samples, max_samples):
            names = wanted or set(positions) | set(efforts)
            rows.append({
                "wall_ns": int(stamp),
                "positions": {name: positions[name] for name in names if name in positions},
                "efforts": {name: efforts[name] for name in names if name in efforts},
            })
        return rows

    def get_object_pose_window(
        self,
        start_wall_ns: int,
        end_wall_ns: int,
        object_id: str,
        max_samples: int = 200,
    ) -> list[dict[str, Any]]:
        with self.lock:
            samples = list(self.object_pose_samples.get(object_id, ()))
        bounded = [s for s in samples if start_wall_ns <= int(s[0]) <= end_wall_ns]
        return [self._pose_tuple_to_dict(s) for s in _uniform_downsample(bounded, max_samples)]

    def get_events_window(self, start_wall_ns: int, end_wall_ns: int) -> list[dict[str, Any]]:
        with self.lock:
            return [
                dict(event)
                for event in self.events
                if start_wall_ns <= _stamp(event) <= end_wall_ns
            ]

    def latest_events(self, limit: int = 100) -> list[dict[str, Any]]:
        with self.lock:
            return [dict(event) for event in list(self.events)[-limit:]]

    def latest_joint_state(self) -> dict[str, Any] | None:
        with self.lock:
            if not self.joint_samples:
                return None
            stamp, positions, efforts = self.joint_samples[-1]
            return {"wall_ns": int(stamp), "positions": dict(positions), "efforts": dict(efforts)}

    def latest_object_pose(self, object_id: str) -> dict[str, Any] | None:
        with self.lock:
            samples = self.object_pose_samples.get(object_id)
            if not samples:
                return None
            return self._pose_tuple_to_dict(samples[-1])

    def pop_wait_events(
        self, event_types: set[str] | None = None
    ) -> list[dict[str, Any]]:
        matched: list[dict[str, Any]] = []
        with self.lock:
            remaining: collections.deque[dict[str, Any]] = collections.deque(maxlen=1000)
            while self._wait_events:
                event = self._wait_events.popleft()
                if event_types is None or event.get("event_type") in event_types:
                    matched.append(event)
                else:
                    remaining.append(event)
            self._wait_events = remaining
        return matched

    def counts(self) -> dict[str, Any]:
        with self.lock:
            return {
                "joint_states": len(self.joint_samples),
                "events": len(self.events),
                "object_poses": {
                    oid: len(samples)
                    for oid, samples in self.object_pose_samples.items()
                },
            }

    def current_step(self) -> str | None:
        with self.lock:
            return self.current_step_id

    def tf_counters(self) -> dict[str, int]:
        with self.lock:
            return dict(self.tf_staleness_counters)

    def object_ids(self) -> list[str]:
        return list(self.object_frames)

    def _pose_tuple_to_dict(self, sample: tuple[Any, ...]) -> dict[str, Any]:
        stamp, object_id, x, y, z, qx, qy, qz, qw = sample
        return {
            "wall_ns": int(stamp),
            "object_id": object_id,
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": float(z)},
                "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
            },
        }


__all__ = [
    "EvidenceRecorder",
    "MAX_JOINT_SAMPLES",
    "MAX_TF_SAMPLES",
    "TF_SAMPLE_HZ",
    "_uniform_downsample",
]
