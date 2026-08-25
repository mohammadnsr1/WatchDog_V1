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
    # HaltClient.halt()'s full return dict ({"ok", "detail", "goals_cancelled"})
    # when halt_on_fire triggered a cancel attempt; None when this fire was
    # flag-only (halt_on_fire=False). action_taken alone cannot say whether a
    # halt actually stopped anything active -- this is that detail.
    halt_result: dict[str, Any] | None = None


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


class RunVerdictPayload(BaseModel):
    """WatchDog conformance verdict scoped to the whole run, not one step.

    Emitted by record_run_verdict for the end-of-run rollup a clean run
    produces instead of a verdict per step. Deliberately a separate payload
    rather than a sentinel step_id on the frozen VerdictPayload: a run-scoped
    verdict has no step_id, and faking one would put it in the same namespace
    as real step verdicts where no consumer could tell them apart.

    Every clause_results entry carries its own step_id, because clause_id is
    "<section>.<field>" (see contracts/loader.py's _make_clause) and is
    therefore identical across all four parts -- without per-entry step_id a
    combined rollup cannot say which part a clause result belongs to.

    steps_covered is a real field rather than free text in explanation so
    coverage can be checked by set comparison against the task's steps: block.
    """

    model_config = ConfigDict(frozen=True)

    verdict: str
    claimed_outcome: str | None
    clause_results: list[dict[str, Any]]
    steps_covered: list[str]
    explanation: str
    run_scope: bool = True


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

    scope is one of "run", "step", "phase", or "abort". The "abort" scope is
    not a readiness signal at all but its inverse -- it declares the run
    should not continue, and travels the same service so the cell has a single
    channel from the monitor. It appears here so an abort is on the timeline
    alongside the readiness signals.
    """

    model_config = ConfigDict(frozen=True)

    scope: str
    step_id: str | None = None
    phase: str | None = None


class FramesCapturedPayload(BaseModel):
    """WatchDog preserved buffered camera frames to the session record.

    Emitted by capture_frames so a visual-evidence capture appears on the
    shared timeline alongside every other event. WatchDog's own output: it is
    in _SELF_EMITTED_EVENT_TYPES and must not wake the agent. The frames
    themselves live on disk under output_dir; this payload only records that a
    capture happened, for which window, and how much it covered.
    """

    model_config = ConfigDict(frozen=True)

    output_dir: str
    frame_count: int
    requested_start_wall_ns: int
    requested_end_wall_ns: int
    covered_start_wall_ns: int | None = None
    covered_end_wall_ns: int | None = None
    label: str | None = None
    step_id: str | None = None


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
        "run_verdict",
        "phase",
        "watchdog_ready",
        "frames_captured",
    ]
    timestamp_ns: int
    wall_time_iso: str
    payload: dict[str, Any]
    subscriber_count: int = 0
    seq: int = 0


# Which side of the executor/WatchDog boundary publishes each event type.
# Used by available_event_types() (watchdog_mcp_server.py) so the monitoring
# agent can tell, for any type it might filter wait_for_event on, whether it
# is something the executor did or something WatchDog itself already knows
# it did.
_EXECUTOR_EVENT_TYPES = frozenset(
    {"begin_step", "end_step", "skill_start", "skill_end", "phase"}
)
_WATCHDOG_EVENT_TYPES = frozenset(
    {
        "monitor_fire", "monitor_armed", "monitor_disarmed", "halt", "verdict",
        "run_verdict", "watchdog_ready", "frames_captured",
    }
)

# event_type -> its payload model. The single source _available_event_types()
# and KNOWN_EVENT_TYPES are built from, so neither can drift from the actual
# schema the way a hand-written list could.
EVENT_TYPE_PAYLOAD_CLASSES: dict[str, type[BaseModel]] = {
    "begin_step": BeginStepPayload,
    "end_step": EndStepPayload,
    "skill_start": SkillStartPayload,
    "skill_end": SkillEndPayload,
    "monitor_fire": MonitorFirePayload,
    "monitor_armed": MonitorArmedPayload,
    "monitor_disarmed": MonitorDisarmedPayload,
    "halt": HaltPayload,
    "verdict": VerdictPayload,
    "run_verdict": RunVerdictPayload,
    "phase": PhasePayload,
    "watchdog_ready": WatchDogReadyPayload,
    "frames_captured": FramesCapturedPayload,
}

# The event_type vocabulary itself, read off EventEnvelope's own Literal
# rather than repeated as a separate list -- one place that can add a type,
# not two that can drift apart.
KNOWN_EVENT_TYPES: tuple[str, ...] = tuple(
    EventEnvelope.model_fields["event_type"].annotation.__args__
)


def event_type_origin(event_type: str) -> str:
    """"executor" or "watchdog": which side publishes this event type."""
    if event_type in _EXECUTOR_EVENT_TYPES:
        return "executor"
    if event_type in _WATCHDOG_EVENT_TYPES:
        return "watchdog"
    raise ValueError(f"unknown event_type {event_type!r}")


def available_event_type_definitions() -> list[dict[str, Any]]:
    """Build available_event_types()'s response from the schema itself.

    Derives name, one-line description, payload fields, and origin from
    EVENT_TYPE_PAYLOAD_CLASSES and KNOWN_EVENT_TYPES -- never a hand-written
    list, so it cannot drift from what the schema actually declares the way
    a wait_for_event(event_types=["skill_phase"]) typo (there is no
    skill_phase; the real type is "phase") was able to.
    """
    definitions = []
    for event_type in KNOWN_EVENT_TYPES:
        payload_cls = EVENT_TYPE_PAYLOAD_CLASSES[event_type]
        doc = (payload_cls.__doc__ or "").strip()
        description = doc.splitlines()[0].strip() if doc else ""
        payload_fields = {
            name: getattr(field.annotation, "__name__", None) or str(field.annotation)
            for name, field in payload_cls.model_fields.items()
        }
        definitions.append(
            {
                "event_type": event_type,
                "description": description,
                "payload_fields": payload_fields,
                "origin": event_type_origin(event_type),
            }
        )
    return definitions


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
        "run_verdict",
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
        | RunVerdictPayload
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
