"""Pydantic event schema for the WatchDog execution event bus.

All events share EventEnvelope. Payload classes are typed helpers used at
CREATION time; at rest (ROS topic / JSONL log file) the payload is a plain dict
so consumers need only standard JSON tooling.

CONSUMER NOTE — timestamp segmentation
---------------------------------------
To segment a continuous sensor stream (e.g. an Isaac Sim RGB camera at 30 Hz)
into per-step or per-skill time windows:

  1. Sort the event log by timestamp_ns (already ordered; file is append-only).
  2. Find begin_step / end_step pairs that share the same step_id.
  3. All sensor frames whose acquire_time_ns falls in the half-open interval
     [begin_step.timestamp_ns, end_step.timestamp_ns) belong to that step window.
  4. skill_start / skill_end pairs WITHIN a step window give sub-windows —
     e.g. frames captured during skill_name == "pick_object" and the
     close_gripper sub-step can be isolated to analyse actual grasp contact.

timestamp_ns is time.monotonic_ns() — monotonic within a process, sub-ms
precision, immune to wall-clock adjustments. Use it for duration arithmetic and
ordering.

wall_time_iso is UTC ISO-8601 — use it to align event log entries against ROS
bag timestamps, Isaac Sim recordings, or any external time base.
"""

from __future__ import annotations

import time
import uuid
from datetime import datetime, timezone
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict


class BeginStepPayload(BaseModel):
    """Agent declares intent to begin a named execution step.

    Names the objects in scope for this step. Does NOT carry a specification
    snapshot — the WatchDog loads contracts independently from the registry
    via its own get_contract tool, so the executor is never the authority on
    which spec the WatchDog checks against.
    """

    model_config = ConfigDict(frozen=True)

    step_id: str
    object_ids: list[str] = []
    notes: str = ""


class EndStepPayload(BaseModel):
    """Agent claims an outcome for a previously begun step.

    claimed_outcome is the agent's self-assessment — not a verified result.
    The WatchDog uses begin_step + end_step timestamps to select the sensor
    window, then independently verifies the claim against observations.
    """

    model_config = ConfigDict(frozen=True)

    step_id: str
    claimed_outcome: Literal["success", "failure", "partial"]
    claim_detail: dict[str, Any] | None = None
    notes: str = ""


class SkillStartPayload(BaseModel):
    """Auto-emitted at the entry of a decorated skill call."""

    model_config = ConfigDict(frozen=True)

    skill_name: str
    params: dict[str, Any]


class SkillEndPayload(BaseModel):
    """Auto-emitted at the exit of a decorated skill call."""

    model_config = ConfigDict(frozen=True)

    skill_name: str
    duration_ns: int
    ok: bool
    error: str | None = None


class MonitorArmedPayload(BaseModel):
    """WatchDog monitor arm event."""

    model_config = ConfigDict(frozen=True)

    monitor_id: str
    monitor_type: str
    step_id: str
    params: dict[str, Any]
    clause: dict[str, Any] | None
    halt_on_fire: bool


class MonitorFirePayload(BaseModel):
    """WatchDog monitor violation event."""

    model_config = ConfigDict(frozen=True)

    monitor_id: str
    monitor_type: str
    step_id: str
    trigger_wall_ns: int
    triggering_value: float
    threshold: float
    comparison: str
    clause: dict[str, Any] | None
    action_taken: str


class MonitorDisarmedPayload(BaseModel):
    """WatchDog monitor disarm event."""

    model_config = ConfigDict(frozen=True)

    monitor_id: str
    step_id: str
    reason: str


class HaltPayload(BaseModel):
    """WatchDog halt event."""

    model_config = ConfigDict(frozen=True)

    step_id: str
    monitor_id: str
    detail: str


class VerdictPayload(BaseModel):
    """WatchDog conformance verdict event."""

    model_config = ConfigDict(frozen=True)

    step_id: str
    verdict: str
    claimed_outcome: str | None
    clause_results: list[dict[str, Any]]
    explanation: str


class PhasePayload(BaseModel):
    """Executor declares a named mid-step phase has been reached.

    Emitted by mark_phase when the executor establishes a physical
    relationship the rest of the step depends on (e.g. closing the gripper
    on an object). step_id is read from the current step state, not passed
    by the caller, so a phase event can never be misattributed to the wrong
    step.
    """

    model_config = ConfigDict(frozen=True)

    step_id: str
    phase: str
    detail: dict[str, Any] | None = None


class WatchDogReadyPayload(BaseModel):
    """WatchDog publishes a readiness signal, for the record.

    This is the topic-side echo of the /cell/signal_ready service call --
    the service call is what actually releases a blocked request_go; this
    event exists only so the signal appears in the same timeline as
    everything else.
    """

    model_config = ConfigDict(frozen=True)

    scope: str
    step_id: str | None = None
    phase: str | None = None


class EventEnvelope(BaseModel):
    """Top-level wrapper shared by all event types.

    Fields
    ------
    event_id         : UUID4 — deduplication key; stable ordering within one process.
    event_type       : discriminator — one of the WatchDog event types.
    timestamp_ns     : time.monotonic_ns() at emission — monotonic, sub-ms precision.
    wall_time_iso    : ISO-8601 UTC wall clock — for cross-process / cross-tool alignment.
    payload          : typed payload serialized to plain dict for storage and transport.
    subscriber_count : ROS publisher's subscription count at publish time (0 if no ROS
                       publisher is configured). A begin_step recorded with
                       subscriber_count: 0 is a provable drop, not an inferred one.
    seq              : monotonic per-process sequence number, starting at 1. event_id
                       (a UUID) identifies an event but cannot reveal a gap; seq can --
                       a consumer that sees 7, 8, 10 knows 9 was lost. Scoped to the
                       process, not the topic: two processes both publish seq=1.
    """

    model_config = ConfigDict(frozen=True)

    event_id: str
    event_type: Literal[
        "begin_step",
        "end_step",
        "skill_start",
        "skill_end",
        "monitor_fire",
        "monitor_armed",
        "monitor_disarmed",
        "halt",
        "verdict",
        "phase",
        "watchdog_ready",
    ]
    timestamp_ns: int
    wall_time_iso: str
    payload: dict[str, Any]
    subscriber_count: int = 0
    seq: int = 0


def make_event(
    event_type: Literal[
        "begin_step",
        "end_step",
        "skill_start",
        "skill_end",
        "monitor_fire",
        "monitor_armed",
        "monitor_disarmed",
        "halt",
        "verdict",
        "phase",
        "watchdog_ready",
    ],
    payload: (
        BeginStepPayload
        | EndStepPayload
        | SkillStartPayload
        | SkillEndPayload
        | MonitorFirePayload
        | MonitorArmedPayload
        | MonitorDisarmedPayload
        | HaltPayload
        | VerdictPayload
        | PhasePayload
        | WatchDogReadyPayload
    ),
) -> EventEnvelope:
    """Create a timestamped EventEnvelope from a typed payload model.

    subscriber_count and seq are left at their defaults here -- EventBus.publish
    fills them in at publish time, since only the bus knows the live
    subscriber count and the process-scoped sequence counter.
    """
    return EventEnvelope(
        event_id=str(uuid.uuid4()),
        event_type=event_type,
        timestamp_ns=time.monotonic_ns(),
        wall_time_iso=datetime.now(timezone.utc).isoformat(),
        payload=payload.model_dump(mode="json"),
    )
