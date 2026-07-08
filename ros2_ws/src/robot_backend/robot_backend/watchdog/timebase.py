"""Wall-clock time helpers for cross-process WatchDog correlation."""

from __future__ import annotations

import time


def wall_ns() -> int:
    """Return process-independent wall-clock nanoseconds."""

    return time.time_ns()
