"""EventBus: fire-and-forget publisher to a ROS 2 topic and a local JSONL log.

One-way only — no acknowledgment, no blocking, no gating of callers.
Any publish error is silently dropped so the skill execution path is never
interrupted by observability infrastructure.

Singleton pattern
-----------------
configure_bus(node, log_path) — create/replace the singleton; call from MCP server init.
get_bus()                     — retrieve the singleton (no-op bus if not configured).
shutdown_bus()                — close file handles; call from MCP server shutdown.

The skill decorator calls get_bus() to retrieve the singleton without needing
dependency injection through every call frame.

ROS topic
---------
/watchdog/events — std_msgs/String, payload is a single-line JSON EventEnvelope.
Kept as String (not a custom message) so consumers need no robot_backend build
dependency: any Python process with rclpy can subscribe and parse.

Log file
--------
One JSON line per event (JSONL). Line-buffered so events survive crashes.
Path: log_path argument, or WATCHDOG_EVENT_LOG env var, or passed explicitly.
"""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Any

from .schema import (
    BeginStepPayload,
    EndStepPayload,
    EventEnvelope,
    GraspWidthSource,
    SkillEndPayload,
    SkillStartPayload,
    make_event,
)

_BUS: EventBus | None = None


class EventBus:
    """Publish EventEnvelope instances to a ROS topic and/or a JSONL log file.

    Pass node=None to disable ROS publishing.
    Pass log_path=None to disable file logging.
    Both can be active simultaneously.
    """

    def __init__(
        self,
        node: Any | None = None,
        log_path: Path | str | None = None,
    ) -> None:
        self._publisher: Any = None
        self._String: Any = None
        self._log_file: Any = None

        if node is not None:
            try:
                from std_msgs.msg import String  # type: ignore[import]
                self._String = String
                self._publisher = node.create_publisher(String, "/watchdog/events", 10)
            except Exception as exc:
                warnings.warn(
                    f"EventBus: ROS publisher creation failed — {exc}. "
                    "Events will only be written to the log file.",
                    stacklevel=2,
                )

        if log_path is not None:
            try:
                resolved = Path(log_path)
                resolved.parent.mkdir(parents=True, exist_ok=True)
                self._log_file = resolved.open("a", encoding="utf-8", buffering=1)
            except Exception as exc:
                warnings.warn(
                    f"EventBus: failed to open log file {log_path!r} — {exc}. "
                    "Events will only be published to the ROS topic.",
                    stacklevel=2,
                )

    def publish(self, event: EventEnvelope) -> None:
        """Publish one event to all configured destinations. Non-blocking."""
        json_str = event.model_dump_json()

        if self._publisher is not None:
            try:
                msg = self._String()
                msg.data = json_str
                self._publisher.publish(msg)
            except Exception:
                pass  # never let observability break execution

        if self._log_file is not None:
            try:
                self._log_file.write(json_str + "\n")
            except Exception:
                pass

    def close(self) -> None:
        """Close the log file handle."""
        if self._log_file is not None:
            try:
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None

    # ------------------------------------------------------------------
    # Typed emit helpers — used by the decorator and the MCP tools
    # ------------------------------------------------------------------

    def emit_begin_step(
        self,
        step_id: str,
        contract: dict[str, Any] | None = None,
    ) -> None:
        self.publish(make_event("begin_step", BeginStepPayload(step_id=step_id, contract=contract)))

    def emit_end_step(
        self,
        step_id: str,
        claimed_outcome: str,
        notes: str = "",
    ) -> None:
        self.publish(make_event(
            "end_step",
            EndStepPayload(
                step_id=step_id,
                claimed_outcome=claimed_outcome,  # type: ignore[arg-type]
                notes=notes,
            ),
        ))

    def emit_skill_start(
        self,
        skill_name: str,
        params: dict[str, Any],
        grasp_width_m: float | None = None,
        grasp_width_source: str | None = None,
    ) -> None:
        source = GraspWidthSource(grasp_width_source) if grasp_width_source else None
        self.publish(make_event(
            "skill_start",
            SkillStartPayload(
                skill_name=skill_name,
                params=params,
                grasp_width_m=grasp_width_m,
                grasp_width_source=source,
            ),
        ))

    def emit_skill_end(
        self,
        skill_name: str,
        duration_ns: int,
        ok: bool,
        error: str | None = None,
    ) -> None:
        self.publish(make_event(
            "skill_end",
            SkillEndPayload(
                skill_name=skill_name,
                duration_ns=duration_ns,
                ok=ok,
                error=error,
            ),
        ))


# ------------------------------------------------------------------
# Module-level singleton
# ------------------------------------------------------------------

def configure_bus(
    node: Any | None = None,
    log_path: Path | str | None = None,
) -> EventBus:
    """Create (or replace) the module-level singleton EventBus.

    Call once from the MCP server's init path after the ROS node has been
    created. All subsequent get_bus() calls return this instance.
    """
    global _BUS
    if _BUS is not None:
        _BUS.close()
    _BUS = EventBus(node=node, log_path=log_path)
    return _BUS


def get_bus() -> EventBus:
    """Return the active bus, or a no-op bus if configure_bus() was not called."""
    if _BUS is None:
        return EventBus()  # no-op: no ROS, no file — safe for tests and import-only contexts
    return _BUS


def shutdown_bus() -> None:
    """Close the singleton bus and clear it (call from atexit / MCP server shutdown)."""
    global _BUS
    if _BUS is not None:
        _BUS.close()
        _BUS = None
