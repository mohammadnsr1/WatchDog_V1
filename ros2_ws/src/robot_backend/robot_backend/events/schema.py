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
from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict


class GraspWidthSource(str, Enum):
    """Provenance of the grasp_width_m value used in a pick_object call.

    REGISTRY_SPEC  — the agent used the constant from model_registry.yaml spec.
    AGENT_ESTIMATE — the agent chose a value not sourced from the registry;
                     the WatchDog can later compare this estimate against the
                     registry spec constant to flag deviation.
    """

    REGISTRY_SPEC = "registry_spec"
    AGENT_ESTIMATE = "agent_estimate"


class BeginStepPayload(BaseModel):
    """Agent declares intent to begin a named execution step.

    contract is the serialized PickContract (or None). Capturing it here
    binds the spec snapshot to the step timestamp so the WatchDog can compare
    observed behaviour against the spec AS IT WAS at execution time, even if
    the registry is later updated.
    """

    model_config = ConfigDict(frozen=True)

    step_id: str
    contract: dict[str, Any] | None = None


class EndStepPayload(BaseModel):
    """Agent claims an outcome for a previously begun step.

    claimed_outcome is the agent's self-assessment — not a verified result.
    The WatchDog uses begin_step + end_step timestamps to select the sensor
    window, then independently verifies the claim against observations.
    """

    model_config = ConfigDict(frozen=True)

    step_id: str
    claimed_outcome: Literal["success", "failure", "partial"]
    notes: str = ""


class SkillStartPayload(BaseModel):
    """Auto-emitted at the entry of a decorated skill call.

    grasp_width_m and grasp_width_source carry pick provenance:
      - registry_spec → agent used the registry constant (auditable by spec)
      - agent_estimate → agent chose a value; WatchDog can compare to spec
    Both fields are None for non-grasp skills.
    """

    model_config = ConfigDict(frozen=True)

    skill_name: str
    params: dict[str, Any]
    grasp_width_m: float | None = None
    grasp_width_source: GraspWidthSource | None = None


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


class EventEnvelope(BaseModel):
    """Top-level wrapper shared by all event types.

    Fields
    ------
    event_id      : UUID4 — deduplication key; stable ordering within one process.
    event_type    : discriminator — one of the WatchDog event types.
    timestamp_ns  : time.monotonic_ns() at emission — monotonic, sub-ms precision.
    wall_time_iso : ISO-8601 UTC wall clock — for cross-process / cross-tool alignment.
    payload       : typed payload serialized to plain dict for storage and transport.
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
    ]
    timestamp_ns: int
    wall_time_iso: str
    payload: dict[str, Any]


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
    ),
) -> EventEnvelope:
    """Create a timestamped EventEnvelope from a typed payload model."""
    return EventEnvelope(
        event_id=str(uuid.uuid4()),
        event_type=event_type,
        timestamp_ns=time.monotonic_ns(),
        wall_time_iso=datetime.now(timezone.utc).isoformat(),
        payload=payload.model_dump(mode="json"),
    )
