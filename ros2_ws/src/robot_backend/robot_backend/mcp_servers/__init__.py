"""MCP servers that expose high-level robot_backend skills to agents."""

from __future__ import annotations

import os
import sys
from pathlib import Path
import site


def _append_system_dist_packages() -> None:
    """Append Ubuntu system dist-packages after the virtualenv site-packages."""

    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        Path(f"/usr/local/lib/python{version}/dist-packages"),
        Path("/usr/lib/python3/dist-packages"),
        Path(f"/usr/lib/python{version}/dist-packages"),
    ]
    for candidate in candidates:
        candidate_str = str(candidate)
        if candidate.is_dir() and candidate_str not in sys.path:
            site.addsitedir(candidate_str)




def _find_preferred_venv_python() -> Path | None:
    """Find a workspace-local virtualenv interpreter for MCP servers."""

    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        candidate = Path(venv) / "bin" / "python3"
        if candidate.is_file():
            return candidate.absolute()

    search_roots = [Path(__file__).resolve(), Path.cwd().resolve()]
    for root in search_roots:
        for parent in (root, *root.parents):
            candidate = parent / ".venv" / "bin" / "python3"
            if candidate.is_file():
                return candidate.absolute()

    return None


def ensure_mcp_server_environment(module_name: str) -> None:
    """Re-exec the current MCP server with the workspace virtualenv when needed."""

    if os.environ.get("_ROBOT_BACKEND_MCP_VENV_READY") == "1":
        _append_system_dist_packages()
        return

    target_python = _find_preferred_venv_python()
    if target_python is None:
        return

    venv_root = str(target_python.parent.parent)
    current_python = Path(sys.executable).absolute()
    if current_python == target_python:
        os.environ.setdefault("VIRTUAL_ENV", venv_root)
        os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")
        _append_system_dist_packages()
        return

    env = os.environ.copy()
    env["VIRTUAL_ENV"] = venv_root
    env["_ROBOT_BACKEND_MCP_VENV_READY"] = "1"

    path_entries = [entry for entry in env.get("PATH", "").split(os.pathsep) if entry]
    venv_bin = str(target_python.parent)
    path_entries = [entry for entry in path_entries if entry != venv_bin]
    env["PATH"] = os.pathsep.join([venv_bin, *path_entries])


    os.execve(
        str(target_python),
        [str(target_python), "-m", module_name, *sys.argv[1:]],
        env,
    )
