"""record_observation(): structured scene observations, shared across hosts.

An observation is an agent's structured, on-the-record note that something in
the scene is unexpected: an object that is not in the registry, a registry
object that moved or went missing, or anything else off-script. It is
deliberately NOT a verdict and NOT a finding: recording one never touches the
finding bookkeeping (_note_finding / findings_during_run / recovered), and
presence of something unexpected is not by itself a violation of anything. If
the observed thing also breaks a requirement, that is a separate, normal
record_verdict call.

This module is the one shared implementation, mirroring vision/look.py's
shape: a Deps dataclass carrying everything the hosting server must provide, a
pure decision core that is unit-testable without FastMCP or ROS, and a store
that gives observations identity so they can be updated and resolved.

Hosts
-----
Today the ONLY host is the WatchDog MCP server, in every scenario.

S4 HOOK: the executor-side observation host planned for Stage 4 will construct
ObservationDeps(role="executor", ...) around the scene server's event bus and
step state, exactly as the WatchDog server does with role="watchdog" -- the
role field on the payload, the per-role look registry below, and the
role-based self-emission rule in watchdog/recorder.py are already in place for
it. Do not register the tool on the scene server before that stage.

Look-id validation
------------------
evidence_look_ids must name looks the recording role actually performed in
this process. vision/look.py registers every completed look's id here (see
register_look), so validation is a set lookup that cannot race the event
topic: a look that happened milliseconds ago is already registered, because
registration happens in the same call that returned the frames.
"""

from __future__ import annotations

import math
import threading
import uuid
from dataclasses import dataclass
from typing import Any, Callable

from robot_backend.watchdog.timebase import wall_ns

OBSERVATION_CATEGORIES = (
    "unlisted_object", "object_moved", "object_missing", "other_unexpected",
)
OBSERVATION_CONFIDENCES = ("low", "medium", "high")
OBSERVATION_STATUSES = ("new", "update", "resolved")
REQUIREMENT_IMPACTS = ("none", "violates", "unknown")

# --------------------------------------------------------------------------
# Per-process look registry: role -> ordered ids of looks that completed.
# Fed by vision/look.py at the moment a look returns frames, so it is the
# in-process ground truth for "looks this role actually made in this session"
# -- unlike scanning the event topic, it cannot miss a look whose event
# publish failed or has not looped back yet.
# --------------------------------------------------------------------------
_LOOK_REGISTRY_LOCK = threading.Lock()
_LOOK_REGISTRY: dict[str, list[str]] = {}


def register_look(role: str, look_id: str) -> None:
    """Record that `role` completed a look with this id. Never raises."""
    try:
        with _LOOK_REGISTRY_LOCK:
            ids = _LOOK_REGISTRY.setdefault(str(role), [])
            if look_id not in ids:
                ids.append(str(look_id))
    except Exception:
        pass


def known_look_ids(role: str) -> list[str]:
    """The ids of every look this role has completed in this process, in order."""
    with _LOOK_REGISTRY_LOCK:
        return list(_LOOK_REGISTRY.get(str(role), []))


def reset_look_registry() -> None:
    """Clear the registry. For tests only -- a server process never resets it."""
    with _LOOK_REGISTRY_LOCK:
        _LOOK_REGISTRY.clear()


class ObservationStore:
    """Thread-safe observation records, keyed by server-generated id.

    One per hosting server process. Holds the latest record for each
    observation so update/resolved calls can be validated against an id that
    actually exists, and so the result of an update can say what it updated.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._records: dict[str, dict[str, Any]] = {}
        self._seq = 0

    def new_id(self) -> str:
        with self._lock:
            self._seq += 1
            return f"obs-{self._seq:03d}-{uuid.uuid4().hex[:8]}"

    def get(self, observation_id: str) -> dict[str, Any] | None:
        with self._lock:
            record = self._records.get(observation_id)
            return dict(record) if record is not None else None

    def put(self, observation_id: str, record: dict[str, Any]) -> None:
        with self._lock:
            self._records[observation_id] = dict(record)

    def ids(self) -> list[str]:
        with self._lock:
            return list(self._records)


@dataclass
class ObservationDeps:
    """Everything record_observation needs from whichever server hosts it.

    role            -- "watchdog" today; "executor" on the future S4 host.
    store           -- this server's ObservationStore.
    publish_event   -- called with one EventEnvelope, or None to skip
                       publishing (unit tests, hosts with no bus).
    current_step_id -- callable returning the step in progress, or None.
    look_ids        -- callable returning the ids of looks THIS role has made;
                       defaults to the module registry vision/look.py feeds.
    logger          -- optional ROS logger for non-fatal complaints.
    """

    role: str
    store: ObservationStore
    publish_event: Callable[[Any], None] | None = None
    current_step_id: Callable[[], str | None] | None = None
    look_ids: Callable[[], list[str]] | None = None
    logger: Any | None = None

    def known_look_ids(self) -> list[str]:
        if self.look_ids is not None:
            return list(self.look_ids())
        return known_look_ids(self.role)


def _finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _validate_location(
    location: Any, category: str, valid_looks: list[str]
) -> dict[str, Any] | None:
    """Return the normalised location dict, or raise ValueError.

    Two forms, either or both in one dict:
      world form: {"frame": "world", "x": <m>, "y": <m>, "z": <m>}
      pixel form: {"pixel": {"u": <px>, "v": <px>}, "look_id": "<id>"}

    At least one form is required unless category is "object_missing" -- a
    thing that is gone has no location to give.
    """
    if location is None:
        if category == "object_missing":
            return None
        raise ValueError(
            "location is required unless category is 'object_missing'. Give "
            '{"frame": "world", "x": ..., "y": ..., "z": ...} and/or '
            '{"pixel": {"u": ..., "v": ...}, "look_id": "<a look you made>"}.'
        )
    if not isinstance(location, dict):
        raise ValueError("location must be a dict (or null for object_missing).")

    has_world = "frame" in location or any(k in location for k in ("x", "y", "z"))
    has_pixel = "pixel" in location or "look_id" in location
    if not has_world and not has_pixel:
        raise ValueError(
            "location carries neither the world form (frame/x/y/z) nor the "
            "pixel form (pixel/look_id). Give at least one."
        )

    normalised: dict[str, Any] = {}
    if has_world:
        frame = location.get("frame")
        if frame != "world":
            raise ValueError(
                f"location.frame must be 'world' (got {frame!r}); world is the "
                "only frame observations are recorded in."
            )
        for axis in ("x", "y", "z"):
            if not _finite_number(location.get(axis)):
                raise ValueError(
                    f"location.{axis} must be a finite number in metres "
                    f"(got {location.get(axis)!r})."
                )
        normalised.update(
            frame="world",
            x=float(location["x"]), y=float(location["y"]), z=float(location["z"]),
        )
    if has_pixel:
        pixel = location.get("pixel")
        if not isinstance(pixel, dict) or not _finite_number(pixel.get("u")) \
                or not _finite_number(pixel.get("v")):
            raise ValueError(
                'location.pixel must be {"u": <px>, "v": <px>} with finite '
                f"numbers (got {pixel!r})."
            )
        look_id = location.get("look_id")
        if not isinstance(look_id, str) or not look_id:
            raise ValueError(
                "the pixel form of location requires look_id: the pixel is "
                "meaningless without the image it indexes into."
            )
        if look_id not in valid_looks:
            raise ValueError(
                f"location.look_id {look_id!r} does not name a look this role "
                f"made in this session. Valid look ids: {valid_looks or 'none yet'}."
            )
        normalised["pixel"] = {"u": float(pixel["u"]), "v": float(pixel["v"])}
        normalised["look_id"] = look_id
    return normalised


def record_observation(
    deps: ObservationDeps,
    summary: str,
    category: str,
    location: dict[str, Any] | None,
    first_seen_wall_ns: int | None,
    evidence_look_ids: list[str],
    evidence_monitor_ids: list[str] | None = None,
    identity: str | None = None,
    confidence: str = "medium",
    requirement_impact: str = "none",
    impact_detail: str | None = None,
    observation_id: str | None = None,
    status: str = "new",
) -> dict[str, Any]:
    """Validate, publish, and store one observation. Returns a tool-result dict.

    Shared by every host so the validation and the event are identical
    whichever server recorded the observation. Never raises: a bad argument
    becomes {"ok": False, "error": ...}, same shape as the MCP servers'
    _error().
    """
    command = "record_observation"

    def error(message: str, **extra: Any) -> dict[str, Any]:
        return {"ok": False, "command": command, "error": message, **extra}

    try:
        if not isinstance(summary, str) or not summary.strip():
            return error("summary must be a non-empty string.")
        if category not in OBSERVATION_CATEGORIES:
            return error(
                f"category must be one of {list(OBSERVATION_CATEGORIES)} "
                f"(got {category!r})."
            )
        if confidence not in OBSERVATION_CONFIDENCES:
            return error(
                f"confidence must be one of {list(OBSERVATION_CONFIDENCES)} "
                f"(got {confidence!r})."
            )
        if status not in OBSERVATION_STATUSES:
            return error(
                f"status must be one of {list(OBSERVATION_STATUSES)} "
                f"(got {status!r})."
            )
        if requirement_impact not in REQUIREMENT_IMPACTS:
            return error(
                f"requirement_impact must be one of {list(REQUIREMENT_IMPACTS)} "
                f"(got {requirement_impact!r})."
            )
        if requirement_impact == "violates" and (
            not isinstance(impact_detail, str) or not impact_detail.strip()
        ):
            return error(
                "requirement_impact='violates' requires impact_detail naming "
                "the requirement it violates. And a violation needs its own "
                "record_verdict as well -- this observation does not count as one."
            )

        valid_looks = deps.known_look_ids()

        if evidence_look_ids is None:
            evidence_look_ids = []
        if not isinstance(evidence_look_ids, list) or any(
            not isinstance(i, str) for i in evidence_look_ids
        ):
            return error("evidence_look_ids must be a list of look-id strings.")
        unknown = [i for i in evidence_look_ids if i not in valid_looks]
        if unknown:
            return error(
                f"evidence_look_ids {unknown} do not name looks this role made "
                f"in this session. Valid look ids: {valid_looks or 'none yet'}."
            )

        monitor_ids = list(evidence_monitor_ids or [])
        if any(not isinstance(i, str) for i in monitor_ids):
            return error("evidence_monitor_ids must be a list of id strings.")

        try:
            normalised_location = _validate_location(location, category, valid_looks)
        except ValueError as exc:
            return error(str(exc))

        if status == "new":
            if observation_id is not None:
                return error(
                    "observation_id is server-generated on status='new'; do not "
                    "pass one. To change an existing observation, use "
                    "status='update' or 'resolved' with its id."
                )
            observation_id = deps.store.new_id()
            previous = None
        else:
            if not isinstance(observation_id, str) or not observation_id:
                return error(
                    f"status={status!r} requires observation_id naming an "
                    "existing observation."
                )
            previous = deps.store.get(observation_id)
            if previous is None:
                return error(
                    f"observation_id {observation_id!r} does not exist. Known "
                    f"observation ids: {deps.store.ids() or 'none yet'}."
                )

        step_id = None
        if deps.current_step_id is not None:
            try:
                step_id = deps.current_step_id()
            except Exception:
                step_id = None

        from robot_backend.events.schema import ObservationPayload, make_event

        now = wall_ns()
        payload = ObservationPayload(
            observation_id=observation_id,
            summary=summary.strip(),
            category=category,
            location=normalised_location,
            first_seen_wall_ns=(
                int(first_seen_wall_ns) if first_seen_wall_ns is not None else None
            ),
            evidence_look_ids=list(evidence_look_ids),
            evidence_monitor_ids=monitor_ids,
            identity=identity,
            confidence=confidence,
            requirement_impact=requirement_impact,
            impact_detail=impact_detail,
            status=status,
            role=deps.role,
            wall_ns=now,
            step_id=step_id,
        )
        event = make_event("observation", payload)

        notes: list[str] = []
        if deps.publish_event is not None:
            try:
                deps.publish_event(event)
            except Exception as exc:
                notes.append(
                    f"observation event not published: {type(exc).__name__}: {exc}"
                )
                if deps.logger is not None:
                    try:
                        deps.logger.warning(
                            f"record_observation: event publish failed: {exc}"
                        )
                    except Exception:
                        pass

        record = payload.model_dump(mode="json")
        deps.store.put(observation_id, record)

        if not evidence_look_ids:
            notes.append(
                "this observation cites no look and is therefore unsupported "
                "by any visual evidence on the record."
            )

        result: dict[str, Any] = {
            "ok": True,
            "command": command,
            "observation_id": observation_id,
            "status": status,
            "unsupported_by_looks": not evidence_look_ids,
            "event": event.model_dump(mode="json"),
            "notes": notes,
        }
        if previous is not None:
            result["previous_status"] = previous.get("status")
        return result
    except Exception as exc:  # the tool must never raise to the model
        return error(f"{type(exc).__name__}: {exc}")


__all__ = [
    "OBSERVATION_CATEGORIES",
    "OBSERVATION_CONFIDENCES",
    "OBSERVATION_STATUSES",
    "REQUIREMENT_IMPACTS",
    "ObservationDeps",
    "ObservationStore",
    "known_look_ids",
    "record_observation",
    "register_look",
    "reset_look_registry",
]
