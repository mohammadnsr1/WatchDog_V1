"""One server binary serves S1, S2 and S3; WATCHDOG_SCENARIO decides which
tools it registers, and main() refuses to serve without a valid scenario.

The scenario is read at import, so each case here re-imports the module under
a patched environment. Everything is asserted against what FastMCP actually
registered -- the module-scope functions exist in every scenario by design
(so unit tests can call them), and registration is the thing the gate
controls.
"""

from __future__ import annotations

import importlib
import os

import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402

# Tools whose registration varies by scenario.
LIBRARY_TOOLS = {"arm_monitor", "available_monitor_definitions"}
AUTHORED_TOOLS = {"arm_custom_monitor"}
# A sample of the ungated surface that must be present in every scenario.
ALWAYS_TOOLS = {
    "get_rendered_task", "get_contract", "get_execution_state",
    "wait_for_event", "record_verdict", "record_run_verdict",
    "disarm_monitor", "disarm_all_monitors", "get_evidence",
}

pytestmark = pytest.mark.skipif(
    not ws._MCP_SERVABLE,
    reason="FastMCP is not importable; registration cannot be observed.",
)


def _reload_with_scenario(monkeypatch, scenario):
    if scenario is None:
        monkeypatch.delenv("WATCHDOG_SCENARIO", raising=False)
    else:
        monkeypatch.setenv("WATCHDOG_SCENARIO", scenario)
    return importlib.reload(ws)


@pytest.fixture(autouse=True)
def _restore_module():
    """Leave the module in its import-time state for every other test file.

    Autouse fixtures are set up before the test's own monkeypatch, so this
    teardown runs after monkeypatch has already restored the environment --
    the final reload therefore re-imports under the original env.
    """
    yield
    importlib.reload(ws)


def _registered_tool_names(module):
    manager = getattr(module.mcp, "_tool_manager", None)
    assert manager is not None, "FastMCP tool manager not found"
    return {tool.name for tool in manager.list_tools()}


def test_s1_registers_library_but_not_authored(monkeypatch):
    module = _reload_with_scenario(monkeypatch, "S1")
    names = _registered_tool_names(module)
    assert LIBRARY_TOOLS <= names
    assert not (AUTHORED_TOOLS & names)
    assert ALWAYS_TOOLS <= names


def test_s2_registers_both(monkeypatch):
    module = _reload_with_scenario(monkeypatch, "S2")
    names = _registered_tool_names(module)
    assert LIBRARY_TOOLS <= names
    assert AUTHORED_TOOLS <= names
    assert ALWAYS_TOOLS <= names


def test_s3_registers_authored_but_not_library(monkeypatch):
    module = _reload_with_scenario(monkeypatch, "S3")
    names = _registered_tool_names(module)
    assert not (LIBRARY_TOOLS & names)
    assert AUTHORED_TOOLS <= names
    assert ALWAYS_TOOLS <= names


def test_gated_functions_exist_at_module_scope_in_every_scenario(monkeypatch):
    for scenario in (None, "S1", "S2", "S3"):
        module = _reload_with_scenario(monkeypatch, scenario)
        assert callable(module.arm_monitor)
        assert callable(module.arm_custom_monitor)
        assert callable(module.available_monitor_definitions)


def test_main_refuses_an_unset_scenario(monkeypatch):
    module = _reload_with_scenario(monkeypatch, None)
    with pytest.raises(RuntimeError, match="WATCHDOG_SCENARIO"):
        module.main()


def test_main_refuses_an_invalid_scenario(monkeypatch):
    module = _reload_with_scenario(monkeypatch, "S9")
    with pytest.raises(RuntimeError, match="WATCHDOG_SCENARIO"):
        module.main()


def test_valid_scenarios_are_exactly_s1_s2_s3(monkeypatch):
    module = _reload_with_scenario(monkeypatch, "S1")
    assert module._VALID_SCENARIOS == {"S1", "S2", "S3"}
