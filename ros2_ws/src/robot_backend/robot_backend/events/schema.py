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


class RunEndedPayload(BaseModel):
    """Executor declares that it has stopped working, and why.

    Emitted exactly once, on EVERY executor-initiated stop: normal
    completion, an unrecoverable fault, a jam, giving up, or a step the cell
    declined. The one path that does NOT emit it is a hard crash (process
    killed, segfault) -- that is what the WatchDog's no-event timeout
    backstops. Before this event existed the WatchDog could only INFER the
    run had ended from silence, which cost a full timeout cycle on every
    ending, clean ones included.

    status is the executor's OWN CLAIM about the whole run, exactly as
    end_step.claimed_outcome is its claim about one step. The WatchDog uses
    it for two things only: to know the run has STOPPED, and to log the
    claim so the scorer can compare it against the WatchDog's independent
    verdict. It must NEVER feed that verdict -- "aborted" does not imply
    non_conformant, and "completed" does not imply conformant. A run where
    the executor claims "completed" and the WatchDog finds non_conformant is
    a run-level independence result, not a contradiction to reconcile.
    """

    model_config = ConfigDict(frozen=True)

    status: Literal["completed", "aborted"]
    reason: str = ""
    detail: dict[str, Any] | None = None


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
    params: dict[str, Any]
    clause: dict[str, Any] | None
    halt_on_fire: bool

    # Structural attribution of the arm to an object, resolved the same way
    # MonitorFirePayload.object_id is: the monitor's own subject
    # (relative_frame's child_frame, stationarity's and region's object_id),
    # or for joint-scoped effort_event the part held at ARM time. None means
    # genuinely no object. params still carries the raw parameters; this is
    # the field to join arms and fires on without knowing each family's
    # parameter names.
    object_id: str | None = None

    # The threshold this monitor will actually compare against, lifted out of
    # params so it is readable without per-family knowledge. None when the
    # family has no scalar threshold, or when it is not finite.
    threshold: float | None = None
    comparison: str = ""

    # --- threshold provenance -------------------------------------------
    # Where the threshold above actually came from. Monitors are armed with
    # AGENT-SUPPLIED params: the server does not read the registry to build
    # them. So "the watchdog read the registry" is a claim about a particular
    # arm call, and these four fields are the evidence for it, established by
    # comparing the passed value against the registry at arm time.
    #
    # threshold_source is the dotted registry path ONLY when the passed value
    # actually matches what the registry holds for this object and field
    # (e.g. "raw_spec.tolerances.grasp_slip_tolerance_m"). Otherwise it names
    # what happened instead -- "agent_literal", "object_unresolved",
    # "no_registry_field", "structural", "unresolved". It must never assert a
    # registry path for a value that did not come from there: the field is
    # only worth anything to a scorer if it can distinguish a genuine
    # registry read from a default that happens to coincide.
    threshold_source: str = "unresolved"
    # True only when the comparison above succeeded. Redundant with
    # threshold_source by construction, and deliberately so: a scorer should
    # not have to parse a string to learn whether verification passed.
    threshold_verified: bool = False
    # Which registry field WAS checked, kept even on a mismatch so the audit
    # trail shows what the value was measured against rather than only that
    # it failed.
    threshold_registry_field: str | None = None
    # What the registry actually held, so a mismatch shows the delta. On a
    # Class 4 run this is the CORRUPTED value, and a monitor armed from it
    # verifies true against it -- which is exactly how arming-from-the-
    # corrupted-registry is proven.
    threshold_registry_value: float | None = None

    # The reference captured at arm time for families that freeze one
    # (relative_frame's parent->child translation, stationarity's position).
    # Empty for families that do not. Previously returned only in the arm
    # tool result, so it was absent from the event stream the scorer reads.
    reference_values: dict[str, Any] = {}

    # --- authored monitors ------------------------------------------------
    # All defaulted so every existing log line still parses. origin says
    # whether this monitor is a library primitive ("library") or agent-written
    # code ("authored"). The rest are null on library arms: code_sha256 hashes
    # the authored source, description is the author's one-line statement of
    # what the monitor checks, channels_read lists the context methods (and
    # first arguments) the authored code actually called, and
    # param_provenance carries the per-param registry verification entries
    # ({name, value, source, verified, registry_value}) for every param whose
    # declared source is a raw_spec path.
    origin: str = "library"
    code_sha256: str | None = None
    description: str | None = None
    channels_read: list[str] | None = None
    param_provenance: list[dict[str, Any]] | None = None


class MonitorFirePayload(BaseModel):
    """WatchDog monitor violation event."""

    model_config = ConfigDict(frozen=True)

    monitor_id: str
    monitor_type: str
    trigger_wall_ns: int
    triggering_value: float
    threshold: float
    comparison: str
    clause: dict[str, Any] | None
    # Structural attribution of the fire to an object, resolved by the engine
    # and independent of whatever the agent put in `clause`. For
    # relative_frame/stationarity/region it is the monitor's own subject; for
    # effort_event, which is joint-scoped and has no intrinsic object, it is
    # the part the gripper was holding at fire time. None means genuinely no
    # object -- an effort fire on an empty gripper, which is itself a finding.
    object_id: str | None = None
    # Provenance of `threshold` above, carried over verbatim from the arm so a
    # fire is auditable on its own without joining back to the arm event. Same
    # semantics and same vocabulary as MonitorArmedPayload -- see the long note
    # there. Copied at arm time rather than re-derived at fire time: the
    # question is where the armed value came from, and re-resolving against a
    # registry that could have been reloaded in between would answer a
    # different question.
    threshold_source: str = "unresolved"
    threshold_verified: bool = False
    threshold_registry_field: str | None = None
    threshold_registry_value: float | None = None
    action_taken: str
    # HaltClient.halt()'s full return dict ({"ok", "detail", "goals_cancelled"})
    # when halt_on_fire triggered a cancel attempt; None when this fire was
    # flag-only (halt_on_fire=False). action_taken alone cannot say whether a
    # halt actually stopped anything active -- this is that detail.
    halt_result: dict[str, Any] | None = None

    # --- authored monitors ------------------------------------------------
    # Same fields and semantics as MonitorArmedPayload's authored block, all
    # defaulted so every existing log line still parses. detail is the
    # optional string the firing check() returned beside its value.
    origin: str = "library"
    code_sha256: str | None = None
    description: str | None = None
    channels_read: list[str] | None = None
    detail: str | None = None


class MonitorDisarmedPayload(BaseModel):
    """WatchDog monitor disarm event."""

    model_config = ConfigDict(frozen=True)

    monitor_id: str
    reason: str


class HaltPayload(BaseModel):
    """WatchDog halt event."""

    model_config = ConfigDict(frozen=True)

    monitor_id: str
    detail: str
    # The step under way when the halt landed, from the recorder's own
    # begin_step/end_step tracking -- never from the halting monitor, which has
    # no step affiliation. None when no step was open, or when the halt came
    # from a monitor fire: such a halt is located in time by the event's
    # wall_time_iso, the same key every other monitor event uses.
    step_id: str | None = None


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

    Emitted by record_run_verdict at termination. EVERY run produces exactly
    one of these, whether or not step findings were recorded along the way.
    The two are complementary, not alternatives: step verdicts are the
    TIMELINE (what went wrong and when), this is the END-STATE assessment
    (did the run ultimately succeed).

    The cadence used to suppress this whenever a finding had been recorded, on
    the reasoning that a rollup would double-count. It does not double-count,
    and the suppression destroyed a real distinction: a run that misseated a
    part and then recovered it looked identical to a run that ended broken --
    one non_conformant step verdict and nothing else in both cases. Nothing in
    the record said whether the run ended good or bad.

    Deliberately a separate payload
    rather than a sentinel step_id on the frozen VerdictPayload: a run-scoped
    verdict has no step_id, and faking one would put it in the same namespace
    as real step verdicts where no consumer could tell them apart.

    Every clause_results entry carries its own step_id, because clause_id is
    "<section>.<field>" (see contracts/loader.py's _make_clause) and is
    therefore identical across all four parts -- without per-entry step_id a
    combined rollup cannot say which part a clause result belongs to.

    steps_covered is a real field rather than free text in explanation so
    coverage can be checked by set comparison against the task's steps: block.

    executor_terminal_status is the status the executor put on its run_ended
    event ("completed"/"aborted"), or None if none arrived (hard crash, so
    the WatchDog terminated on its no-event timeout). It is recorded BESIDE
    verdict, never folded into it: the scorer's whole run-level independence
    measure is the cross-product of the two, and "executor said completed,
    WatchDog said non_conformant" is its most interesting cell. Kept separate
    from claimed_outcome, which is the executor's per-step self-assessment
    and answers a different question.

    findings_during_run and recovered join this end-state assessment to the
    timeline. Both are CODE-populated from the record_verdict calls the server
    actually saw, never from an argument, for the same reason
    executor_terminal_status is: an agent that forgets a finding it recorded
    would otherwise be able to report a clean recovery that never happened.
    """

    model_config = ConfigDict(frozen=True)

    verdict: str
    claimed_outcome: str | None
    clause_results: list[dict[str, Any]]
    steps_covered: list[str]
    explanation: str
    run_scope: bool = True
    executor_terminal_status: str | None = None

    # step_ids where a step-scoped finding was recorded during the run, in the
    # order recorded, de-duplicated. Empty on a clean run. Lets a scorer join
    # the final verdict to the timeline without re-parsing the event stream.
    findings_during_run: list[str] = []
    # True when findings occurred AND this final verdict is conformant: the run
    # went wrong and was put right. A distinct, correctly-labeled outcome --
    # neither a plain conformant run nor a failed one -- and the case the whole
    # cadence change exists to make visible. False on a clean run (nothing to
    # recover from) and on a run that ended broken.
    recovered: bool = False


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
    # The serving scenario (WATCHDOG_SCENARIO), stamped on every readiness
    # signal so the session record says which tool surface was live.
    # Defaulted so pre-scenario logs still parse.
    scenario: str | None = None


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
        "run_ended",
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
    {"begin_step", "end_step", "run_ended", "skill_start", "skill_end", "phase"}
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
    "run_ended": RunEndedPayload,
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
    # Kept in step with EventEnvelope.event_type above. "frames_captured" was
    # missing here while being a valid envelope type, so make_event(
    # "frames_captured", ...) type-checked wrong though it passed at runtime.
    event_type: Literal[
        "begin_step",
        "end_step",
        "run_ended",
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
    ],
    payload: (
        BeginStepPayload
        | EndStepPayload
        | RunEndedPayload
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
        | FramesCapturedPayload
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
