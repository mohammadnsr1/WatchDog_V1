"""Wall-clock time for the orchestrator's own JSONL log.

Deliberately duplicated (see agent_harness/timebase.py and
robot_backend/watchdog/timebase.py, which duplicate this same three-liner for
the same reason): the orchestrator is a standalone node with no dependency on
either the LLM harness or robot_backend, and must stay that way.
"""

import time


def wall_ns() -> int:
    return time.time_ns()
