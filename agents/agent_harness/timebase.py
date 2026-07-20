"""Wall-clock time for correlating agent turns with cell events.

Deliberately duplicated rather than imported from robot_backend: the harness
must not depend on ros2_ws. Must return time.time_ns() so every transcript
shares one axis with the executor's own events and WatchDog's monitor fires.
"""

import time


def wall_ns() -> int:
    return time.time_ns()
