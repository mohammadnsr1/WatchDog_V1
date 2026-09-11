"""Provenance fields in get_rendered_task's response (FIX 3).

get_rendered_task already re-reads the task from disk on every call, so the
content is never stale. The path it reads, however, comes from
EXECUTOR_TASK_PATH / WATCHDOG_TASK_PATH -- and a stale env var serves an OLD
task perfectly freshly and completely invisibly. These tests pin the fields
that make a wrong-file or wrong-registry run visible in the session's first
turn instead of never.

Pure Python -- no cell, no rclpy. Run with:
    pytest test/test_role_context_provenance.py -v
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from robot_backend.mcp_servers import role_context as rc


REPO_ROOT = Path(__file__).resolve().parents[4]
TASKS_DIR = REPO_ROOT / "agents" / "tasks"


@pytest.fixture
def rendered(monkeypatch, tmp_path):
    """Render the executor task with the session dir pinned to a tmp path.

    resolve_session_dir_strict refuses to run without a real session dir (it
    guards against two sessions independently resolving 'fresh'), so the
    override stands in for a live cell.
    """
    monkeypatch.setenv("WATCHDOG_SESSION_DIR", str(tmp_path))

    def _render(task_path: Path):
        monkeypatch.setenv("EXECUTOR_TASK_PATH", str(task_path))
        return rc.build_rendered_task("executor")

    return _render


def _requires(task_name: str) -> Path:
    path = TASKS_DIR / task_name
    if not path.is_file():
        pytest.skip(f"{path} not present in this checkout")
    return path


def test_response_carries_resolved_task_path_and_registry_sha(rendered):
    task = _requires("fmb_assembly_2.executor.yaml")
    sources = rendered(task)["sources"]

    assert sources["task_path"] == str(task.resolve())
    assert sources["task_path_env"] == "EXECUTOR_TASK_PATH"

    sha = sources["registry_sha256"]
    assert sha is not None and len(sha) == 64
    # The reported hash must be of the file the tool also names, not some
    # other registry copy -- otherwise the provenance pair is meaningless.
    reported = Path(sources["registry_path"])
    assert sha == hashlib.sha256(reported.read_bytes()).hexdigest()


def test_wrong_task_path_is_visible_in_the_response(rendered):
    """A stale env var must show the file it ACTUALLY loaded.

    This is the whole point of FIX 3: the render succeeds either way, so the
    only thing distinguishing a correct run from a run against last week's
    task file is this field.
    """
    intended = _requires("fmb_assembly_2.executor.yaml")
    wrong = _requires("fmb_assembly.executor.yaml")

    sources = rendered(wrong)["sources"]
    assert sources["task_path"] == str(wrong.resolve())
    assert sources["task_path"] != str(intended.resolve())


def test_task_path_is_absolute_even_when_env_var_is_relative(rendered, monkeypatch):
    """A relative env var resolves to an absolute path in the response.

    Two processes with different cwds would otherwise report the same relative
    string for different files.
    """
    task = _requires("fmb_assembly_2.executor.yaml")
    monkeypatch.chdir(REPO_ROOT)
    sources = rendered(Path("agents/tasks/fmb_assembly_2.executor.yaml"))["sources"]

    assert Path(sources["task_path"]).is_absolute()
    assert sources["task_path"] == str(task.resolve())
    # The raw env var is echoed alongside, so the resolution is auditable.
    assert sources["task_path_env_value"] == "agents/tasks/fmb_assembly_2.executor.yaml"


def test_live_registry_provenance_never_raises(monkeypatch):
    """Both agents call this at startup; it must degrade, not block the run."""
    monkeypatch.setattr(
        rc.session_helper, "sha256_file",
        lambda _p: (_ for _ in ()).throw(OSError("unreadable")),
    )
    path, sha = rc._live_registry_provenance()
    assert path is not None
    assert sha is None
