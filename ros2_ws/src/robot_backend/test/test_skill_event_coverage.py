"""Regression guard: every executor MCP tool must be visible on /watchdog/events.

The monitoring layer's mid-step arming design and any post-hoc analysis both
depend on observing the executor's skill stream. A tool that emits nothing is
invisible to both. This test enumerates every tool actually registered on
each executor MCP server's FastMCP instance (not a hand-maintained list, so it
cannot silently drift from the real tool set) and asserts each one is either:

  1. Decorated with @emit_skill_events directly (or wraps a function that is,
     since functools.wraps preserves the marker attribute through any number
     of pass-through layers reachable via __wrapped__), or
  2. Named in an explicit allowlist below, with a documented reason.

A tool with neither is a coverage gap and fails the test. This is what stops
the gap reopening when a new tool is added months from now.

Run with: pytest test/test_skill_event_coverage.py -v
"""

from __future__ import annotations

import functools

# ---------------------------------------------------------------------------
# Explicit allowlists — every entry must say *why* it is exempt from direct
# @emit_skill_events decoration at the tool level.
# ---------------------------------------------------------------------------

SCENE_ALLOWLIST: dict[str, str] = {
    "get_scene_summary": "delegates to get_scene_summary_skill, which is decorated",
    "reset_object_state": "delegates to reset_object_state_skill, which is decorated",
    "recover_scene": "delegates to recover_scene_skill, which is decorated",
    "set_permitted_contacts": "delegates to set_permitted_contacts_skill, which is decorated",
    "begin_step": "emits a dedicated begin_step event directly, not skill_start/skill_end",
    "end_step": "emits a dedicated end_step event directly, not skill_start/skill_end",
    "mark_phase": "emits a dedicated phase event directly, not skill_start/skill_end",
}

GRIPPER_ALLOWLIST: dict[str, str] = {
    "open_gripper": "delegates to open_gripper_skill, which is decorated",
    "close_gripper": "delegates to close_gripper_skill, which is decorated",
    "set_gripper_width": "delegates to set_gripper_width_skill, which is decorated",
    "grasp": "delegates to grasp_skill, which is decorated",
    "release": "delegates to release_skill, which is decorated",
}

MOTION_ALLOWLIST: dict[str, str] = {
    "compute_grasp_approach_poses": (
        "pure local computation (pose geometry only) -- touches no backend, "
        "no ROS service, no physical state"
    ),
}


def _is_covered(fn) -> bool:
    """True if fn (or anything in its __wrapped__ chain) was @emit_skill_events."""
    seen = set()
    current = fn
    while current is not None and id(current) not in seen:
        if getattr(current, "_emits_skill_events", False):
            return True
        seen.add(id(current))
        current = getattr(current, "__wrapped__", None)
    return False


def uncovered_tools(mcp_instance, allowlist: dict[str, str]) -> list[str]:
    """Return tool names that are neither decorated nor allowlisted."""
    gaps = []
    for tool in mcp_instance._tool_manager.list_tools():
        if tool.name in allowlist:
            continue
        if not _is_covered(tool.fn):
            gaps.append(tool.name)
    return gaps


# ---------------------------------------------------------------------------
# Real servers
# ---------------------------------------------------------------------------

class TestSceneServerCoverage:
    def test_all_tools_covered(self) -> None:
        from robot_backend.mcp_servers import scene_mcp_server as srv

        gaps = uncovered_tools(srv.mcp, SCENE_ALLOWLIST)
        assert gaps == [], (
            f"scene MCP tool(s) with no event coverage and no allowlist entry: {gaps}"
        )

    def test_allowlist_has_no_stale_entries(self) -> None:
        from robot_backend.mcp_servers import scene_mcp_server as srv

        registered = {t.name for t in srv.mcp._tool_manager.list_tools()}
        stale = set(SCENE_ALLOWLIST) - registered
        assert stale == set(), f"allowlist references tools no longer registered: {stale}"


class TestGripperServerCoverage:
    def test_all_tools_covered(self) -> None:
        from robot_backend.mcp_servers import gripper_mcp_server as srv

        gaps = uncovered_tools(srv.mcp, GRIPPER_ALLOWLIST)
        assert gaps == [], (
            f"gripper MCP tool(s) with no event coverage and no allowlist entry: {gaps}"
        )

    def test_allowlist_has_no_stale_entries(self) -> None:
        from robot_backend.mcp_servers import gripper_mcp_server as srv

        registered = {t.name for t in srv.mcp._tool_manager.list_tools()}
        stale = set(GRIPPER_ALLOWLIST) - registered
        assert stale == set(), f"allowlist references tools no longer registered: {stale}"


class TestMotionServerCoverage:
    def test_all_tools_covered(self) -> None:
        from robot_backend.mcp_servers import motion_mcp_server as srv

        gaps = uncovered_tools(srv.mcp, MOTION_ALLOWLIST)
        assert gaps == [], (
            f"motion MCP tool(s) with no event coverage and no allowlist entry: {gaps}"
        )

    def test_allowlist_has_no_stale_entries(self) -> None:
        from robot_backend.mcp_servers import motion_mcp_server as srv

        registered = {t.name for t in srv.mcp._tool_manager.list_tools()}
        stale = set(MOTION_ALLOWLIST) - registered
        assert stale == set(), f"allowlist references tools no longer registered: {stale}"


# ---------------------------------------------------------------------------
# The coverage-check logic itself: confirm it actually fails when a decorator
# is missing, so this guard cannot pass vacuously.
# ---------------------------------------------------------------------------

class TestCoverageCheckDetectsGaps:
    def test_undecorated_unlisted_tool_is_flagged(self) -> None:
        class FakeTool:
            def __init__(self, name, fn):
                self.name = name
                self.fn = fn

        class FakeToolManager:
            def __init__(self, tools):
                self._tools = tools

            def list_tools(self):
                return self._tools

        class FakeMCP:
            def __init__(self, tools):
                self._tool_manager = FakeToolManager(tools)

        def covered_fn():
            return {"ok": True}

        covered_fn._emits_skill_events = True

        def uncovered_fn():
            return {"ok": True}

        fake_mcp = FakeMCP(
            [FakeTool("covered", covered_fn), FakeTool("uncovered", uncovered_fn)]
        )

        gaps = uncovered_tools(fake_mcp, allowlist={})
        assert gaps == ["uncovered"]

    def test_allowlisted_tool_is_not_flagged_even_if_undecorated(self) -> None:
        class FakeTool:
            def __init__(self, name, fn):
                self.name = name
                self.fn = fn

        class FakeToolManager:
            def __init__(self, tools):
                self._tools = tools

            def list_tools(self):
                return self._tools

        class FakeMCP:
            def __init__(self, tools):
                self._tool_manager = FakeToolManager(tools)

        def uncovered_fn():
            return {"ok": True}

        fake_mcp = FakeMCP([FakeTool("exempt", uncovered_fn)])

        gaps = uncovered_tools(fake_mcp, allowlist={"exempt": "documented reason"})
        assert gaps == []

    def test_marker_survives_functools_wraps_chain(self) -> None:
        def base():
            return {"ok": True}

        base._emits_skill_events = True

        @functools.wraps(base)
        def outer_wrapper(*args, **kwargs):
            return base(*args, **kwargs)

        assert _is_covered(outer_wrapper) is True
