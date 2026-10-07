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
from robot_backend.registry import executor_registry_override, load_registry

from .timebase import wall_ns

JOINT_STATES_TOPIC = "/joint_states"
EVENTS_TOPIC = "/watchdog/events"
TF_SAMPLE_HZ = 20.0

# A chain whose own latest TF time lags the freshest chain's by more than this
# is excluded from the tick's common-time computation and sampled at its own
# latest instead (see _on_tf_timer): a chain that has stopped publishing must
# not drag every other sample into the past with it.
TF_COMMON_TIME_MAX_LAG_SEC = 1.0


def _tf_header_stamp_ns(transform: Any) -> int:
    """The TF time a lookup result was evaluated at, off its own header."""
    stamp = transform.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

# WatchDog's own output event types, published to the same /watchdog/events
# topic this recorder subscribes to. The recorder hears itself: without this
# exclusion, wait_for_event wakes the agent on its own signal_ready /
# arm_monitor / disarm_monitor / record_verdict calls, wasting a turn on
# activity the agent already knows it just did. These are still appended to
# `events` for the timeline -- only the wake path (_wait_events) excludes
# them. monitor_fire IS in this set: fires reach the agent through the
# engine's fire queue (which also sets the wake event), so queuing the topic
# echo here as well double-delivered every fire -- the copy survived the wait
# that consumed the queued fire and arrived STALE at the next wait (observed
# live 2026-10-01, a T3 fire replayed at the start of T5). halt is here
# because only WatchDog publishes it and the agent already learns of the halt
# through the fire's action_taken/halt_result. frames_captured likewise:
# waking the agent on its own capture would waste a turn.
# monitor_disarmed is here for the agent's OWN disarms and fire echoes, but
# an authored_* disarm (timeout/bad_return/error) is the lane telling the
# agent its monitor DIED -- that is new information and is delivered; see
# _is_self_emitted.
_SELF_EMITTED_EVENT_TYPES = frozenset(
    {
        "watchdog_ready", "monitor_armed", "monitor_disarmed", "monitor_fire",
        "verdict", "run_verdict", "halt", "frames_captured",
    }
)
# monitor_disarmed reasons that are the agent's own doing (or the echo of a
# fire it is already receiving through the queue): suppressed from the wake
# path. Every OTHER reason -- the authored_* failures -- wakes the wait.
_SELF_DISARM_REASONS = frozenset(
    {"fired", "agent_disarm", "agent_disarm_all", "auto_disarm_release",
     "internal_error"}
)
# "look" is deliberately NOT in the set above: it is the one event type BOTH
# roles emit, so type alone cannot say whether this process emitted it.
# _is_self_emitted() decides it from LookPayload.role instead. "observation"
# follows the same rule for the same reason: the WatchDog records
# observations today and a future S4 executor-side host will emit the same
# type, so ObservationPayload.role decides, not the type.

# Types that never wake a bare wait_for_event -- a caller must name them in
# event_types to receive one. Distinct from self-emitted, which is about who
# produced the event; this is about whether it is worth interrupting for.
#
# An executor look is a real action by the other agent and belongs on the
# WatchDog's timeline (it lands in self.events, so get_events returns it). But
# waking the WatchDog every time the executor glanced at the workspace would
# flood its loop with events it has no decision to make about, and the events it
# waits on are the ones that change what it must do. So it is available on
# request and silent otherwise.
# "observation" is opt-in for the same reason "look" is: the WatchDog's own
# observations are suppressed as self-emitted anyway (by role), and a future
# executor-side observation is a note on the timeline, not something the
# WatchDog must be interrupted for -- it is available to a wait that names it.
_WAKE_OPT_IN_EVENT_TYPES = frozenset({"look", "observation"})


def _is_self_emitted(envelope: Any) -> bool:
    """Whether THIS process produced the event, so it must not wake its own wait.

    Event type answers this for every type except "look", which both the
    WatchDog and the executor emit. A WatchDog look is self-emitted -- calling
    look() and then being woken by that same call would spin the loop. An
    executor look is someone else's action and is not suppressed here; it is
    gated instead by _WAKE_OPT_IN_EVENT_TYPES, which lets a caller that asks for
    "look" receive it while a caller that did not is left undisturbed.
    """
    event_type = envelope.event_type
    if event_type in ("look", "observation"):
        # Both types are emitted by both roles (observation: WatchDog today,
        # the S4 executor-side host later), so the payload's role field is
        # the only thing that can say whose it was.
        payload = envelope.payload if isinstance(envelope.payload, dict) else {}
        return payload.get("role") == "watchdog"
    if event_type == "monitor_disarmed":
        # The agent's own disarms (and the echo of a fire it receives through
        # the engine's queue) are suppressed; an authored_* failure disarm is
        # the lane reporting that an armed monitor DIED, which the agent can
        # learn no other way while blocked in wait_for_event -- a budget-test
        # monitor once died 2 s into a 60 s wait and the agent saw nothing
        # (2026-10-01). Delivered, with the payload's reason and detail.
        payload = envelope.payload if isinstance(envelope.payload, dict) else {}
        return str(payload.get("reason", "")) in _SELF_DISARM_REASONS
    return event_type in _SELF_EMITTED_EVENT_TYPES


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
    elif (override := executor_registry_override()) is not None:
        # Class 4: executor-role processes only (the executor_monitor host);
        # in the watchdog server this returns None by its role guard.
        resolved = override
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            resolved = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
        except Exception:
            resolved = _DEFAULT_REGISTRY_PATH
    return load_registry(resolved).raw_dict


def _sampled_object_frames(registry: dict[str, Any]) -> dict[str, dict[str, str]]:
    """Every registry object with a resolvable TF frame, sampled for evidence.

    Filters on `enabled` but NOT on `kind`, and the distinction matters.

    `kind` is the wrong filter here. Excluding kind: frame entries would drop
    every placement, hole and reorient frame from the watchdog's buffers, so
    query_snapshot and get_evidence would return null for them and the seating
    check -- "the part is seated at its placement frame within tolerance", the
    watchdog's central effect -- would have nowhere to get the target except
    the executor's own report. That is the one measurement the independent
    channel exists for. Frames are evidence even though they are not bodies.

    `enabled` is the right filter, now that it means "the stage publishes
    this" rather than "this gets collision geometry". Sampling disabled
    entries was not merely wasteful: signal_ready(scope="run") requires a
    buffered pose for every frame this function returns, so a single authored-
    but-unpublished entry made run-scope readiness permanently unreachable and
    every executor gate paid its full 90s timeout. Two absent cubes and two
    unpublished reorient frames cost a one-part run six minutes on
    2026-09-21. An entry that cannot warm up must therefore not be sampled,
    and `enabled: false` is how the registry declares that.

    An enabled frame that the stage nonetheless fails to publish still yields
    no samples and shows up in tf_staleness_counters, which is the honest
    outcome and is already how a missing part frame behaves -- but it will now
    also hold up run-scope readiness, which is the intended pressure to keep
    `enabled` truthful.
    """
    frames: dict[str, dict[str, str]] = {}
    objects = registry.get("objects", {})
    if not isinstance(objects, dict):
        return frames
    for object_id, entry in objects.items():
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("enabled", True)):
            continue
        tf_frame = entry.get("tf_frame")
        lookup_frame = entry.get("lookup_frame")
        if (
            isinstance(tf_frame, str)
            and tf_frame
            and isinstance(lookup_frame, str)
            and lookup_frame
        ):
            frames[str(object_id)] = {
                "tf_frame": tf_frame,
                "lookup_frame": lookup_frame,
                "kind": str(entry.get("kind", "object")).strip().lower(),
            }
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
        self.object_frames = _sampled_object_frames(self.registry)
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
            # One WALL stamp for the whole tick, shared by every object pose
            # and cell frame sampled in it -- so a relative_frame clause can
            # pair a TCP sample with an object sample by identical wall_ns (a
            # zip) rather than needing to interpolate between mismatched
            # timestamps.
            #
            # And one TF time for the whole tick. A Time(0) lookup resolves
            # each chain at ITS OWN latest TF time, and in this cell the robot
            # chain republishes at ~11 Hz against the Isaac object chains'
            # ~34 Hz, so two latest-time samples from one tick can be up to
            # ~50 ms apart. A relative quantity assembled from two such
            # samples shows motion-induced phantom drift -- measured at
            # 18-26 mm during ordinary approach/lift moves on the 2026-10-02
            # S3 bag, against <= 1.1 mm for a time-consistent lookup, which
            # fired two slip monitors on a grasp that never slipped (see
            # docs/dev/PREBATCH_FIXES.md item 1). So: pass 1 finds each
            # chain's latest TF time, the tick's common time is the LATEST
            # TIME AVAILABLE TO EVERY healthy chain (the minimum of those),
            # and pass 2 samples every chain at that one time. tf_stamp_ns on
            # each sample records the TF time actually used.
            #
            # The common time lags "now" by at most the slowest healthy
            # chain's publish interval (~90 ms at 11 Hz) -- the same order of
            # staleness a Time(0) sample of that slowest chain already had,
            # so detection/halt latency is unchanged; the samples are simply
            # mutually consistent now.
            tick_wall_ns = wall_ns()

            chains: list[tuple[str, dict[str, str], bool]] = [
                (object_id, self.object_frames[object_id], True)
                for object_id in list(self.object_frames)
            ] + [
                (frame_id, self.cell_frames[frame_id], False)
                for frame_id in list(self.cell_frames)
            ]

            # Pass 1: each chain's latest transform and the TF time tf2 used
            # for it (the lookup result's own header stamp). A failed lookup
            # bumps the chain's staleness counter, exactly as before.
            latest: list[tuple[str, dict[str, str], Any, int, bool]] = []
            for ident, frame, is_object in chains:
                try:
                    transform = self.lookup_transform(frame["lookup_frame"], frame["tf_frame"])
                    latest.append(
                        (ident, frame, transform, _tf_header_stamp_ns(transform), is_object)
                    )
                except Exception:
                    with self.lock:
                        self.tf_staleness_counters[ident] = (
                            self.tf_staleness_counters.get(ident, 0) + 1
                        )

            # The tick's common TF time: the latest time available to every
            # chain -- excluding chains lagging the freshest by more than
            # TF_COMMON_TIME_MAX_LAG_SEC. A chain that has stopped publishing
            # must not drag every other sample into the past with it; it is
            # sampled at its own latest (it is already inaccurate, and
            # tf_staleness_counters is the channel that reports it).
            common_ns: int | None = None
            if latest:
                newest = max(stamp for _, _, _, stamp, _ in latest)
                lag_limit = int(TF_COMMON_TIME_MAX_LAG_SEC * 1e9)
                eligible = [
                    stamp for _, _, _, stamp, _ in latest if newest - stamp <= lag_limit
                ]
                if eligible:
                    common_ns = min(eligible)

            # Pass 2: sample every eligible chain at the common time. A chain
            # whose latest IS the common time keeps its pass-1 result; a
            # re-lookup that fails (a chain that only just appeared, with no
            # cache at the common time) falls back to its pass-1 latest, with
            # tf_stamp_ns saying so honestly.
            for ident, frame, transform, stamp, is_object in latest:
                tf_stamp_ns = stamp
                if common_ns is not None and stamp != common_ns and (
                    stamp - common_ns >= 0
                ):
                    try:
                        transform = self.lookup_transform_at(
                            frame["lookup_frame"], frame["tf_frame"], common_ns
                        )
                        tf_stamp_ns = common_ns
                    except Exception:
                        pass
                pose = self._transform_to_pose_tuple(
                    tick_wall_ns, ident, transform, tf_stamp_ns
                )
                with self.lock:
                    buffers = self.object_pose_samples if is_object else self.frame_samples
                    buffers.setdefault(
                        ident, collections.deque(maxlen=self.max_tf_samples)
                    ).append(pose)

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
            is_self_emitted = _is_self_emitted(envelope)
            with self.lock:
                self.events.append(record)
                if not is_self_emitted:
                    self._wait_events.append(record)
                if envelope.event_type == "begin_step" and isinstance(step_id, str):
                    self.current_step_id = step_id
                elif envelope.event_type == "end_step":
                    self.current_step_id = None
            # Mechanical reactions to the executor's own skill events: the
            # engine brackets the held span from grasp/release and takes down
            # a released part's slip monitor before the opening jaws can read
            # as a slip. Kept outside the lock above, and never allowed to
            # break event recording.
            if not is_self_emitted and self.engine is not None:
                payload = envelope.payload if isinstance(envelope.payload, dict) else {}
                skill_name = payload.get("skill_name")
                if isinstance(skill_name, str):
                    try:
                        self.engine.on_skill_event(
                            envelope.event_type,
                            skill_name,
                            payload.get("params"),
                            payload.get("ok"),
                        )
                    except Exception as exc:
                        self._log_warn(f"WatchDog engine skill hook failed: {exc}")

            if not is_self_emitted:
                self.wake_event.set()
        except Exception as exc:
            self._log_warn(f"WatchDog event-tail callback failed: {exc}")

    def _transform_to_pose_tuple(
        self, stamp: int, object_id: str, transform: Any, tf_stamp_ns: int | None = None
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
            # Appended LAST so every existing consumer that unpacks or indexes
            # the first nine fields keeps working; _pose_tuple_to_dict accepts
            # both lengths.
            int(tf_stamp_ns) if tf_stamp_ns is not None else None,
        )

    def lookup_transform(self, parent_frame: str, child_frame: str) -> Any:
        if self.tf_buffer is None:
            raise RuntimeError("TF buffer is not attached.")
        try:
            import rclpy
            return self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
        except Exception:
            return self.tf_buffer.lookup_transform(parent_frame, child_frame, None)

    def lookup_transform_at(
        self, parent_frame: str, child_frame: str, stamp_ns: int
    ) -> Any:
        """Transform at one specific TF time, for the common-time tick sampling."""
        if self.tf_buffer is None:
            raise RuntimeError("TF buffer is not attached.")
        import rclpy
        return self.tf_buffer.lookup_transform(
            parent_frame, child_frame, rclpy.time.Time(nanoseconds=int(stamp_ns))
        )

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

    def observed_terminal_status(self) -> str | None:
        """Status of the last ingested run_ended event, or None if there was none.

        Reads the ingest buffer, which every event enters exactly once when the
        subscriber receives it, rather than any tool's delivery path. That
        distinction is the whole point of this method.

        The status used to be captured only as wait_for_event drained an event,
        which coupled "did the watchdog observe the run end" to "did the agent
        happen to have a blocking wait open at the moment it fired". On the
        abort run of 2026-09-21 it did not: the last wait_for_event had already
        returned, run_ended arrived 69s later, and the agent picked it up with
        get_events -- a path that captured nothing. The verdict's
        executor_terminal_status fell through to the no_terminal_event
        sentinel while the explanation, written from that same get_events
        output, correctly said "aborted". The clean run was right only because
        its run_ended happened to fire 2s into an open wait.

        Reading the buffer at verdict time instead makes the field independent
        of how, when, or whether the agent drained the event.

        Returns the LAST run_ended if somehow several exist: a re-emitted
        terminal event supersedes an earlier one, matching how the drain-time
        capture overwrote its slot.
        """
        with self.lock:
            records = list(self.events)
        for record in reversed(records):
            if record.get("event_type") != "run_ended":
                continue
            payload = record.get("payload")
            if isinstance(payload, dict):
                status = payload.get("status")
                if isinstance(status, str) and status:
                    return status
        return None

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
                event_type = event.get("event_type")
                # One decision, two cases: an explicit event_types list is
                # honoured exactly as given (so naming "look" delivers looks),
                # while an unfiltered wait takes everything EXCEPT the opt-in
                # types. Anything not taken stays queued for a later caller.
                wanted = (
                    event_type in event_types if event_types is not None
                    else event_type not in _WAKE_OPT_IN_EVENT_TYPES
                )
                if wanted:
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
        # 10-tuples carry tf_stamp_ns (the TF time the sample was evaluated
        # at) appended last; 9-tuples are the pre-alignment shape and still
        # parse, reporting tf_stamp_ns: null.
        stamp, ident, x, y, z, qx, qy, qz, qw = sample[:9]
        tf_stamp_ns = sample[9] if len(sample) > 9 else None
        return {
            "wall_ns": int(stamp),
            # Samples from one recorder tick share one TF time (tf_stamp_ns).
            "tf_stamp_ns": int(tf_stamp_ns) if tf_stamp_ns is not None else None,
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
