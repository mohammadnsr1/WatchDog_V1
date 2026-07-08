"""Execution event bus for WatchDog conformance checking.

One-way, structured, timestamped event stream. Publishers only — no
acknowledgment, no blocking, no gating of the skill execution path.

Public API
----------
configure_bus(node, log_path)  — initialise the singleton bus (call once from MCP server)
get_bus()                      — retrieve the active bus (or a no-op bus)
shutdown_bus()                 — close file handles and clear singleton
emit_skill_events              — decorator: wrap a skill to auto-emit skill_start/end
EventEnvelope                  — top-level event model (all events in this shape)
GraspWidthSource               — enum: "registry_spec" | "agent_estimate"
"""

from .bus import configure_bus, get_bus, shutdown_bus
from .decorator import emit_skill_events
from .schema import (
    BeginStepPayload,
    EndStepPayload,
    EventEnvelope,
    GraspWidthSource,
    HaltPayload,
    MonitorArmedPayload,
    MonitorDisarmedPayload,
    MonitorFirePayload,
    SkillEndPayload,
    SkillStartPayload,
    VerdictPayload,
    make_event,
)

__all__ = [
    "configure_bus",
    "get_bus",
    "shutdown_bus",
    "emit_skill_events",
    "EventEnvelope",
    "GraspWidthSource",
    "BeginStepPayload",
    "EndStepPayload",
    "SkillStartPayload",
    "SkillEndPayload",
    "MonitorArmedPayload",
    "MonitorFirePayload",
    "MonitorDisarmedPayload",
    "HaltPayload",
    "VerdictPayload",
    "make_event",
]
