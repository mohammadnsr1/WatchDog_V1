"""EventBus: fire-and-forget publisher to a ROS 2 topic and a local JSONL log.

One-way only — no acknowledgment, no gating of callers. Any publish error is
silently dropped so the skill execution path is never interrupted by
observability infrastructure.

The one exception to "no blocking" is a bounded, one-time wait for a matched
subscriber before this process's very first publish (see
EventBus._wait_for_subscriber_once). This exists because DDS discovery is not
instantaneous: a publish issued shortly after this process's publisher is
constructed can beat discovery matching it to an already-running subscriber,
and default (volatile) QoS retains no history for endpoints that match later
-- the sample is silently discarded. The executor's first event is always
begin_step, which is exactly the message the monitoring layer most needs, so
this is worth a short bounded wait rather than leaving it to chance.
Deliberately NOT solved with TRANSIENT_LOCAL durability: that would fix this
drop but would also replay buffered begin_step events to any later-joining
subscriber, which could arm monitors for steps that have already finished --
replay is more dangerous here than loss.

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

import os
import time
import warnings
from pathlib import Path
from typing import Any

from .schema import (
    BeginStepPayload,
    EndStepPayload,
    EventEnvelope,
    SkillEndPayload,
    SkillStartPayload,
    make_event,
)

_BUS: EventBus | None = None


# Default bound on the first-publish subscriber wait (A1). Overridable via
# WATCHDOG_EVENTBUS_WAIT_SEC for environments where DDS discovery is
# consistently slower or faster than this default.
_DEFAULT_SUBSCRIBER_WAIT_SEC = 2.0
_SUBSCRIBER_POLL_INTERVAL_SEC = 0.02


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
        self._node: Any = node
        self._publisher: Any = None
        self._String: Any = None
        self._log_file: Any = None
        # A1: the bounded subscriber wait runs once per process, before the
        # first publish only -- never per publish. This flag is the guard.
        self._waited_for_subscriber = False
        # A3: per-process monotonic sequence number, starting at 1.
        self._seq = 0

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

    def _wait_for_subscriber_once(self) -> None:
        """Bounded poll for a matched subscriber, run once before the first publish.

        A publish issued milliseconds after this process's publisher is
        constructed can beat DDS discovery matching it to an already-running
        subscriber (e.g. WatchDog's recorder). Default QoS is volatile, so
        that sample is silently discarded -- and the executor's first event
        is always begin_step, the one message the monitoring layer most
        needs. This does not fix late-joining subscribers (nothing can,
        short of TRANSIENT_LOCAL, which trades this problem for a worse one
        -- see the module docstring); it only covers the already-running-but-
        not-yet-matched case, which is what a cold-started publisher process
        actually hits.

        Never raises, never blocks past the timeout: running the executor
        with no monitoring subscriber at all is a supported configuration.
        """
        if self._waited_for_subscriber or self._publisher is None:
            return
        self._waited_for_subscriber = True

        timeout_sec = _DEFAULT_SUBSCRIBER_WAIT_SEC
        override = os.environ.get("WATCHDOG_EVENTBUS_WAIT_SEC")
        if override:
            try:
                timeout_sec = float(override)
            except ValueError:
                pass

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                if self._publisher.get_subscription_count() > 0:
                    return
            except Exception:
                return  # can't introspect subscriber count; don't block on it
            time.sleep(_SUBSCRIBER_POLL_INTERVAL_SEC)

        self._log_warn_once_no_subscriber(timeout_sec)

    def _log_warn_once_no_subscriber(self, timeout_sec: float) -> None:
        message = (
            f"EventBus: no subscriber matched within {timeout_sec:.2f}s of the "
            "first publish; publishing anyway. The first event on this topic "
            "may be lost if a subscriber connects after this point."
        )
        logger = getattr(self._node, "get_logger", None)
        if logger is not None:
            try:
                logger().warning(message)
                return
            except Exception:
                pass
        warnings.warn(message, stacklevel=2)

    def publish(self, event: EventEnvelope) -> None:
        """Publish one event to all configured destinations. Non-blocking (after
        the one-time bounded subscriber wait on the very first publish; see
        _wait_for_subscriber_once)."""
        self._wait_for_subscriber_once()

        subscriber_count = 0
        if self._publisher is not None:
            try:
                subscriber_count = self._publisher.get_subscription_count()
            except Exception:
                subscriber_count = 0

        self._seq += 1
        # event is frozen; model_copy(update=...) returns a new instance
        # rather than mutating it, which is what "frozen" is protecting.
        stamped = event.model_copy(
            update={"subscriber_count": subscriber_count, "seq": self._seq}
        )
        json_str = stamped.model_dump_json()

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
        object_ids: list[str] | None = None,
        notes: str = "",
    ) -> None:
        self.publish(make_event(
            "begin_step",
            BeginStepPayload(step_id=step_id, object_ids=object_ids or [], notes=notes),
        ))

    def emit_end_step(
        self,
        step_id: str,
        claimed_outcome: str,
        claim_detail: dict[str, Any] | None = None,
        notes: str = "",
    ) -> None:
        self.publish(make_event(
            "end_step",
            EndStepPayload(
                step_id=step_id,
                claimed_outcome=claimed_outcome,  # type: ignore[arg-type]
                claim_detail=claim_detail,
                notes=notes,
            ),
        ))

    def emit_skill_start(
        self,
        skill_name: str,
        params: dict[str, Any],
    ) -> None:
        self.publish(make_event(
            "skill_start",
            SkillStartPayload(skill_name=skill_name, params=params),
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
