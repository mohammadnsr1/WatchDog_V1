"""Always-on evidence recorder for the WatchDog node."""

from __future__ import annotations

import collections
import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from robot_backend.events.schema import EventEnvelope
from robot_backend.registry import load_registry

from .timebase import wall_ns

JOINT_STATES_TOPIC = "/joint_states"
EVENTS_TOPIC = "/watchdog/events"
TF_SAMPLE_HZ = 20.0

# WatchDog's own output event types, published to the same /watchdog/events
# topic this recorder subscribes to. The recorder hears itself: without this
# exclusion, wait_for_event wakes the agent on its own signal_ready /
# arm_monitor / disarm_monitor / record_verdict calls, wasting a turn on
# activity the agent already knows it just did. These are still appended to
# `events` for the timeline -- only the wake path (_wait_events) excludes
# them. monitor_fire is deliberately not in this set: it reaches the agent
# through the engine's fire queue, a different path entirely, and must keep
# waking it. halt is in this set for the same reason as the others: only
# WatchDog ever publishes it, and the agent already learns of the halt
# through that same monitor_fire fire (action_taken/halt_result) -- a
# separate wake on its own echo would burn a turn for no new information.
# frames_captured is here for the same reason: capture_frames is WatchDog's own
# output (see FramesCapturedPayload), and waking the agent on its own capture
# would waste a turn on something it just did.
_SELF_EMITTED_EVENT_TYPES = frozenset(
    {
        "watchdog_ready", "monitor_armed", "monitor_disarmed", "verdict", "halt",
        "frames_captured",
    }
)


def _env_int(name: str, default: int) -> int:
    """Read a positive-int override from the environment, or return default.

    A missing, empty, non-integer, or non-positive value falls back to the
    default rather than raising -- a misconfigured env var must not stop the
    recorder from starting.
    """
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return default
    return value if value > 0 else default


# In-memory buffer ceilings. Configurable via env so a run longer than the
# default cannot lose an early window while still running (get_evidence reads
# these deques, not the bag). Defaults sized for ~1 hour: /joint_states was
# measured at ~51 Hz average (with gaps up to 0.333 s under load), TF at 20 Hz.
# Memory cost is tens of MB, negligible beside Isaac Sim. The bag, not these
# buffers, is the complete post-hoc record; these serve live monitoring only.
MAX_JOINT_SAMPLES = _env_int("WATCHDOG_MAX_JOINT_SAMPLES", 216000)  # ~1 h @ ~51 Hz + headroom
MAX_TF_SAMPLES = _env_int("WATCHDOG_MAX_TF_SAMPLES", 72000)  # 1 h @ 20 Hz

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
    return load_registry(resolved).raw_dict


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


def _cell_frame_ids(registry: dict[str, Any]) -> dict[str, dict[str, str]]:
    """frame_id -> {tf_frame, lookup_frame} for every *_frame field under
    cell.gripper (at minimum tcp_frame), looked up against cell.base_frame.

    These are cell-level TF frames, not registry objects -- there is no
    per-object entry for the gripper TCP. Without sampling them the same way
    object poses are sampled, a relative_frame clause over a cell frame (e.g.
    grasp retention, tcp vs an object) can only ever be checked live by the
    monitor itself; get_evidence could never return a trajectory for it, and
    a verdict would have to fall back to "monitor didn't fire" as its only
    evidence -- exactly the "conformance from absence of fire" the system
    prompt forbids, forced by a buffering gap rather than chosen.
    """
    cell = registry.get("cell", {})
    if not isinstance(cell, dict):
        return {}
    base_frame = cell.get("base_frame")
    if not isinstance(base_frame, str) or not base_frame:
        return {}
    gripper = cell.get("gripper", {})
    if not isinstance(gripper, dict):
        return {}
    frames: dict[str, dict[str, str]] = {}
    for key, value in gripper.items():
        if key.endswith("_frame") and isinstance(value, str) and value:
            frames[value] = {"tf_frame": value, "lookup_frame": base_frame}
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


def event_wrapper(record: dict[str, Any]) -> dict[str, Any]:
    """Project an internal event record into the agent-facing event shape.

    Every path that hands an event to an agent (wait_for_event's drain,
    get_events, get_evidence's events channel) uses this, so the agent
    always receives a directly usable wall_ns -- the same field name and
    axis as recorder buffers -- instead of having to find (or guess) it
    inside the envelope. envelope is passed through unmodified for
    provenance; the raw internal record (wall_ns_received, envelope_wall_ns)
    is not part of the agent-facing shape.
    """
    return {
        "wall_ns": record.get("wall_ns", record.get("wall_ns_received")),
        "wall_ns_source": record.get("wall_ns_source", "received"),
        "event_type": record.get("event_type"),
        "envelope": record.get("envelope", record),
    }


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
        self.cell_frames = _cell_frame_ids(self.registry)
        # Read the env-configurable ceilings once, at construction, so a test
        # (or a run) can raise them without re-importing the module. The module
        # constants are the defaults.
        self.max_joint_samples = _env_int("WATCHDOG_MAX_JOINT_SAMPLES", MAX_JOINT_SAMPLES)
        self.max_tf_samples = _env_int("WATCHDOG_MAX_TF_SAMPLES", MAX_TF_SAMPLES)
        self.joint_samples: collections.deque[Any] = collections.deque(
            maxlen=self.max_joint_samples
        )
        self.object_pose_samples: dict[str, collections.deque[Any]] = {
            oid: collections.deque(maxlen=self.max_tf_samples) for oid in self.object_frames
        }
        self.frame_samples: dict[str, collections.deque[Any]] = {
            frame_id: collections.deque(maxlen=self.max_tf_samples)
            for frame_id in self.cell_frames
        }
        self.events: collections.deque[dict[str, Any]] = collections.deque(maxlen=5000)
        self._wait_events: collections.deque[dict[str, Any]] = collections.deque(maxlen=1000)
        # Shared by object ids and cell frame ids -- both are "a TF lookup
        # that just failed", the same kind of staleness regardless of which
        # side of the object/frame distinction it's on.
        self.tf_staleness_counters: dict[str, int] = {oid: 0 for oid in self.object_frames}
        self.tf_staleness_counters.update({fid: 0 for fid in self.cell_frames})
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
            # One stamp for the whole tick, shared by every object pose and
            # cell frame sampled in it -- so a relative_frame clause can pair
            # a TCP sample with an object sample by identical wall_ns (a
            # zip) rather than needing to interpolate between mismatched
            # timestamps.
            tick_wall_ns = wall_ns()
            for object_id in list(self.object_frames):
                frame = self.object_frames[object_id]
                try:
                    transform = self.lookup_transform(frame["lookup_frame"], frame["tf_frame"])
                    pose = self._transform_to_pose_tuple(tick_wall_ns, object_id, transform)
                    with self.lock:
                        self.object_pose_samples.setdefault(
                            object_id, collections.deque(maxlen=self.max_tf_samples)
                        ).append(pose)
                except Exception:
                    with self.lock:
                        self.tf_staleness_counters[object_id] = (
                            self.tf_staleness_counters.get(object_id, 0) + 1
                        )
            for frame_id in list(self.cell_frames):
                frame = self.cell_frames[frame_id]
                try:
                    transform = self.lookup_transform(frame["lookup_frame"], frame["tf_frame"])
                    pose = self._transform_to_pose_tuple(tick_wall_ns, frame_id, transform)
                    with self.lock:
                        self.frame_samples.setdefault(
                            frame_id, collections.deque(maxlen=self.max_tf_samples)
                        ).append(pose)
                except Exception:
                    with self.lock:
                        self.tf_staleness_counters[frame_id] = (
                            self.tf_staleness_counters.get(frame_id, 0) + 1
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
            envelope_wall_ns = _parse_wall_time_iso(envelope.wall_time_iso)
            # wall_ns is the resolved, agent-facing timestamp: the publishing
            # process's own wall clock (parsed from wall_time_iso) when
            # available, falling back to this process's receipt time only if
            # that parse failed. Either way it is on the same wall_ns axis as
            # every recorder buffer (joint/pose samples all use wall_ns()) --
            # unlike envelope.timestamp_ns, a per-process monotonic clock
            # that is not comparable across processes and would silently
            # select an empty evidence window if used for one. See
            # get_evidence and _stamp, which windows on this field.
            resolved_wall_ns = envelope_wall_ns if envelope_wall_ns is not None else received
            wall_ns_source = "envelope" if envelope_wall_ns is not None else "received"
            record = {
                "wall_ns_received": received,
                "envelope_wall_ns": envelope_wall_ns,
                "wall_ns": resolved_wall_ns,
                "wall_ns_source": wall_ns_source,
                "event_type": envelope.event_type,
                "payload": envelope.payload,
                "envelope": envelope_dict,
            }
            step_id = (
                envelope.payload.get("step_id")
                if isinstance(envelope.payload, dict)
                else None
            )
            is_self_emitted = envelope.event_type in _SELF_EMITTED_EVENT_TYPES
            with self.lock:
                self.events.append(record)
                if not is_self_emitted:
                    self._wait_events.append(record)
                if envelope.event_type == "begin_step" and isinstance(step_id, str):
                    self.current_step_id = step_id
                elif envelope.event_type == "end_step":
                    self.current_step_id = None
            if not is_self_emitted:
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

    def get_frame_window(
        self,
        start_wall_ns: int,
        end_wall_ns: int,
        frame_id: str,
        max_samples: int = 200,
    ) -> list[dict[str, Any]]:
        """Mirrors get_object_pose_window, for a cell frame (e.g. tcp)."""
        with self.lock:
            samples = list(self.frame_samples.get(frame_id, ()))
        bounded = [s for s in samples if start_wall_ns <= int(s[0]) <= end_wall_ns]
        return [
            self._pose_tuple_to_dict(s, id_key="frame_id")
            for s in _uniform_downsample(bounded, max_samples)
        ]

    def latest_frame_pose(self, frame_id: str) -> dict[str, Any] | None:
        with self.lock:
            samples = self.frame_samples.get(frame_id)
            if not samples:
                return None
            return self._pose_tuple_to_dict(samples[-1], id_key="frame_id")

    def frame_ids(self) -> list[str]:
        return list(self.cell_frames)

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

    def buffered_range(
        self,
        object_ids: list[str] | None = None,
        frame_ids: list[str] | None = None,
    ) -> dict[str, Any]:
        """Return [first, last] wall_ns currently buffered per channel.

        None for a channel/object/frame with no buffered samples at all. Used
        to tell an out-of-range get_evidence window apart from a legitimate
        "nothing happened here" -- see get_evidence in watchdog_mcp_server.py.
        """
        with self.lock:
            joint_stamps = [int(s[0]) for s in self.joint_samples]
            events_stamps = [_stamp(e) for e in self.events]
            ids = object_ids if object_ids is not None else list(self.object_pose_samples)
            pose_ranges: dict[str, list[int] | None] = {}
            for oid in ids:
                stamps = [int(s[0]) for s in self.object_pose_samples.get(oid, ())]
                pose_ranges[oid] = [stamps[0], stamps[-1]] if stamps else None
            fids = frame_ids if frame_ids is not None else list(self.frame_samples)
            frame_ranges: dict[str, list[int] | None] = {}
            for fid in fids:
                stamps = [int(s[0]) for s in self.frame_samples.get(fid, ())]
                frame_ranges[fid] = [stamps[0], stamps[-1]] if stamps else None
        return {
            "joint_states": [joint_stamps[0], joint_stamps[-1]] if joint_stamps else None,
            "object_poses": pose_ranges,
            "frames": frame_ranges,
            "events": [events_stamps[0], events_stamps[-1]] if events_stamps else None,
        }

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

    def _pose_tuple_to_dict(
        self, sample: tuple[Any, ...], id_key: str = "object_id"
    ) -> dict[str, Any]:
        stamp, ident, x, y, z, qx, qy, qz, qw = sample
        return {
            "wall_ns": int(stamp),
            id_key: ident,
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
    "event_wrapper",
]
