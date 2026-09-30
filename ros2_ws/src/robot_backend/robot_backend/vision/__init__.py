"""Vision surface shared by the WatchDog and executor MCP servers.

Holds the one implementation of look() -- the tool that returns camera frames to
the MODEL as image content, as opposed to capture_frames, which preserves frames
to disk for later analysis and which the agent never sees.

There is one module rather than one per server because the two roles differ in
exactly three things -- the role label, the saved-file prefix, and which event
bus the look event goes onto -- and none of those is a reason to have two
implementations that can drift apart. Everything else, including the staleness
rules and the depth-source fallback, must be identical or the two agents are not
being measured on the same instrument.
"""

from .look import (
    LookDeps,
    LookResult,
    build_look_content,
    build_look_event,
    depth_to_grayscale_png,
    downscale,
    look,
    perform_look,
)

__all__ = [
    "LookDeps",
    "LookResult",
    "build_look_content",
    "build_look_event",
    "depth_to_grayscale_png",
    "downscale",
    "look",
    "perform_look",
]
