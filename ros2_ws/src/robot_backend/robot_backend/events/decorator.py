"""@emit_skill_events: auto-emit skill_start / skill_end without touching skill bodies.

Apply to any skill function. The decorator:
  1. Records the entry timestamp.
  2. Emits a skill_start event with the serializable subset of kwargs.
     For pick_object: grasp_width_m and grasp_width_source are extracted from
     kwargs and promoted to top-level payload fields for WatchDog provenance checks.
  3. Calls the wrapped function.
  4. Emits skill_end with duration_ns and ok/error from the result dict.

The decorator never suppresses exceptions and never blocks — it uses `finally`
so skill_end is guaranteed to fire even on exception paths.

Usage:
    from robot_backend.events.decorator import emit_skill_events

    @emit_skill_events
    def pick_object(backend, *, object_id, grasp_width_m, grasp_width_source="agent_estimate", ...):
        ...
"""

from __future__ import annotations

import functools
import time
from typing import Any, Callable


# Infrastructure params that carry no audit value in event logs and are either
# too large (registry dict) or not JSON-serializable (tf2_ros.Buffer).
_EXCLUDED_PARAMS: frozenset[str] = frozenset({"registry", "tf_buffer"})


def _sanitize_params(kwargs: dict[str, Any]) -> dict[str, Any]:
    """Return the serializable (JSON-safe) subset of a kwargs dict.

    Drops complex objects (RobotBackend, pose dicts with non-primitive values,
    etc.) so the event payload remains pure JSON without importing the full
    type hierarchy on the consumer side.
    """
    safe: dict[str, Any] = {}
    for key, value in kwargs.items():
        if key in _EXCLUDED_PARAMS:
            continue
        if isinstance(value, (str, int, float, bool, type(None))):
            safe[key] = value
        elif isinstance(value, (list, tuple)) and all(
            isinstance(item, (str, int, float, bool, type(None))) for item in value
        ):
            safe[key] = list(value)
        elif isinstance(value, dict):
            # Include only if every leaf value is JSON-safe (shallow check).
            try:
                import json
                json.dumps(value)
                safe[key] = value
            except (TypeError, ValueError):
                safe[key] = f"<non-serializable {type(value).__name__}>"
        # Complex objects (RobotBackend, Buffer, etc.) are intentionally dropped.
    return safe


def emit_skill_events(func: Callable) -> Callable:
    """Decorator: emit skill_start and skill_end events around a skill call.

    Reads grasp_width_m and grasp_width_source from kwargs automatically so
    pick_object provenance lands in skill_start without any change to the skill
    function body.
    """
    from .bus import get_bus

    @functools.wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        bus = get_bus()
        skill_name: str = func.__qualname__
        start_ns: int = time.monotonic_ns()

        # Grasp-width provenance — present only on pick_object calls.
        grasp_width_m: float | None = kwargs.get("grasp_width_m")
        grasp_width_source: str | None = kwargs.get("grasp_width_source")

        bus.emit_skill_start(
            skill_name=skill_name,
            params=_sanitize_params(kwargs),
            grasp_width_m=grasp_width_m,
            grasp_width_source=grasp_width_source,
        )

        ok: bool = False
        error: str | None = None
        try:
            result = func(*args, **kwargs)
            ok = result.get("ok", True) if isinstance(result, dict) else True
            return result
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            duration_ns: int = time.monotonic_ns() - start_ns
            bus.emit_skill_end(
                skill_name=skill_name,
                duration_ns=duration_ns,
                ok=ok,
                error=error,
            )

    return wrapper
