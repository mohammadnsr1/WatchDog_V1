"""Per-session directory and manifest for the WatchDog cell record.

One run is one directory: what the cell did (bag, orchestrator log), what each
process logged (events_*.jsonl), what each agent decided (transcripts), and the
registry that produced it (manifest). Everything correlates by wall_ns.

The hard part is that the ROS side (cell bringup) and the harness side (agents,
launched separately from a virtualenv) are different process trees that must
agree on ONE directory without the operator wiring it by hand. They agree
through a single resolution order, implemented once here and used by both:

  1. WATCHDOG_SESSION_DIR if set -- an explicit override always wins.
  2. the path in ~/.watchdog/current_session if it exists and names a real
     directory -- the pointer the launch writes so the harness needs no export.
  3. otherwise a fresh session under the root (WATCHDOG_SESSION_ROOT, else
     <repo>/agents/transcripts/sessions/, else ~/.watchdog/sessions/).

The session root lives under the agents' transcripts directory (already
gitignored) so a bag never reaches git. Nothing here is on any hot path.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_SESSION_ENV = "WATCHDOG_SESSION_DIR"
_SESSION_ROOT_ENV = "WATCHDOG_SESSION_ROOT"
_MANIFEST_NAME = "manifest.json"
_POINTER_FILE = Path.home() / ".watchdog" / "current_session"


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def repo_root() -> Path | None:
    """Locate the WatchDog_V1 repo root by walking up for agents/ + ros2_ws/.

    Returns None when neither marker is found -- e.g. an install-only
    deployment where this file lives under install/ and the source tree is
    absent. Callers fall back to ~/.watchdog in that case.
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "agents").is_dir() and (parent / "ros2_ws").is_dir():
            return parent
    return None


def source_registry_path() -> Path | None:
    """Path to the SOURCE model_registry.yaml, or None if the repo isn't found."""
    root = repo_root()
    if root is None:
        return None
    candidate = root / "ros2_ws" / "src" / "robot_backend" / "config" / "model_registry.yaml"
    return candidate if candidate.is_file() else None


def session_root() -> Path:
    """Parent directory new sessions are created under.

    WATCHDOG_SESSION_ROOT overrides. Otherwise <repo>/agents/transcripts/sessions
    -- under the gitignored transcripts dir so bags never reach git. Falls back
    to ~/.watchdog/sessions only when the repo root cannot be located.
    """
    env = os.environ.get(_SESSION_ROOT_ENV)
    if env and env.strip():
        return Path(os.path.expanduser(env.strip()))
    root = repo_root()
    if root is not None:
        return root / "agents" / "transcripts" / "sessions"
    return Path.home() / ".watchdog" / "sessions"


def read_pointer() -> Path | None:
    """Session dir named by ~/.watchdog/current_session, if it names a real dir."""
    try:
        raw = _POINTER_FILE.read_text(encoding="utf-8").strip()
    except Exception:
        return None
    if not raw:
        return None
    candidate = Path(os.path.expanduser(raw))
    return candidate if candidate.is_dir() else None


def write_pointer(session_dir: Path) -> Path:
    """Atomically record the active session dir in ~/.watchdog/current_session.

    Temp file then rename, so a reader never sees a half-written path. Rewritten
    at each bringup. This is what lets the separately-launched agents find the
    cell's session without the operator exporting anything.
    """
    _POINTER_FILE.parent.mkdir(parents=True, exist_ok=True)
    tmp = _POINTER_FILE.with_name(f".{_POINTER_FILE.name}.{os.getpid()}.tmp")
    tmp.write_text(str(session_dir) + "\n", encoding="utf-8")
    os.replace(tmp, _POINTER_FILE)
    return _POINTER_FILE


def resolve_session_dir() -> tuple[Path, str]:
    """Resolve the session dir WITHOUT creating it. Returns (path, rule).

    rule is "env", "pointer", or "fresh" -- so callers can log which rule
    produced the path and an operator can see at a glance whether the two sides
    agree.
    """
    env = os.environ.get(_SESSION_ENV)
    if env and env.strip():
        return Path(os.path.expanduser(env.strip())), "env"
    ptr = read_pointer()
    if ptr is not None:
        return ptr, "pointer"
    return session_root() / _utc_stamp(), "fresh"


def active_session_dir() -> Path | None:
    """The session dir if one is already determined (env or pointer), else None.

    Never creates a fresh session -- used by per-artifact log-path defaults that
    must not diverge into a new directory when no session is active. A process
    with no env and no pointer is genuinely standalone and falls back to legacy
    ~/.watchdog paths at the call site.
    """
    session_dir, rule = resolve_session_dir()
    return session_dir if rule in ("env", "pointer") else None


def ensure_session_dir() -> Path:
    """Resolve the session dir (full order) and create it, tolerating a race."""
    session_dir, _ = resolve_session_dir()
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir


def event_log_default(server_name: str) -> str:
    """Default events_<server_name>.jsonl path for a process.

    Inside the active session dir (env or pointer) when one is determined, else
    the legacy ~/.watchdog/ location. Only the DEFAULT -- an explicitly set
    WATCHDOG_EVENT_LOG / WATCHDOG_MONITOR_LOG env var still wins at the call
    site. The per-server filename keeps the four processes from colliding on one
    shared session path.
    """
    base = active_session_dir() or (Path.home() / ".watchdog")
    return str(base / f"events_{server_name}.jsonl")


def monitor_log_default() -> str:
    """Default monitor_events.jsonl path for the WatchDog server."""
    base = active_session_dir() or (Path.home() / ".watchdog")
    return str(base / "monitor_events.jsonl")


def orchestrator_log_dir_default() -> str:
    """Default orchestrator log directory (session/orchestrator, else legacy)."""
    active = active_session_dir()
    if active is not None:
        return str(active / "orchestrator")
    return str(Path.home() / ".watchdog" / "orchestrator")


def sha256_file(path: Path) -> str:
    """sha256 of a file's raw bytes, as a hex string."""
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def git_commit(cwd: Path | None = None) -> str | None:
    """Return the workspace HEAD commit if obtainable, else None."""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(cwd) if cwd is not None else None,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
    except Exception:
        return None
    commit = out.stdout.strip()
    return commit or None


def write_manifest(
    session_dir: Path,
    *,
    registry: dict[str, Any],
    registry_path: Path | str | None,
    registry_sha256: str | None,
    started_wall_ns: int,
    recorded_topics: list[str],
    images_recorded: bool,
    image_throttle_hz: float,
    record_pointcloud: bool = False,
    git_commit_value: str | None = None,
    registry_source_path: Path | str | None = None,
    registry_source_sha256: str | None = None,
) -> Path:
    """Write manifest.json into the session dir, atomically and idempotently.

    Atomic: written to a temp file then renamed, so a reader never sees a
    half-written manifest. Idempotent: if a manifest already exists (another
    process wrote it first), this leaves it untouched -- the first writer wins,
    and every process computes the same content anyway.

    The embedded registry is the load-bearing part. In fault-injection runs the
    registry IS the injected fault; capturing it (and its hash) alongside the
    recording is what lets a corrupted run be told from a clean one after the
    fact, mechanically, across a batch.

    registry_path/registry_sha256 describe the file the cell actually LOADED
    (the install-space copy). registry_source_path/registry_source_sha256
    describe the SOURCE copy an operator edits. registry_install_matches_source
    is derived: when it is false, the cell is running a different registry than
    the one on disk in src/ -- the classic "edited the source, forgot to
    rebuild" trap that makes a fault-injection run silently execute the clean
    registry. Null source hash (install-only deployment) omits the comparison.
    """
    manifest_path = session_dir / _MANIFEST_NAME
    if manifest_path.exists():
        return manifest_path

    if registry_source_sha256 is not None and registry_sha256 is not None:
        install_matches_source: bool | None = registry_source_sha256 == registry_sha256
    else:
        install_matches_source = None

    session_id = session_dir.name
    manifest = {
        "session_id": session_id,
        "started_wall_ns": int(started_wall_ns),
        "registry": registry,
        "registry_sha256": registry_sha256,
        "registry_path": str(registry_path) if registry_path is not None else None,
        "registry_source_path": (
            str(registry_source_path) if registry_source_path is not None else None
        ),
        "registry_source_sha256": registry_source_sha256,
        "registry_install_matches_source": install_matches_source,
        "clock": {
            "cell_use_sim_time": True,
            "bag_recorder_use_sim_time": False,
            "correlation_field": "recv_timestamp",
            "header_stamp_warning": (
                "Message header.stamp is simulation time, reset to zero each "
                "run. Never use it for correlation. Use the bag's recv_timestamp, "
                "which is system-clock nanoseconds and comparable to wall_ns."
            ),
        },
        "recorded_topics": list(recorded_topics),
        "images_recorded": bool(images_recorded),
        "image_throttle_hz": float(image_throttle_hz),
        "record_pointcloud": bool(record_pointcloud),
        "git_commit": git_commit_value,
    }

    session_dir.mkdir(parents=True, exist_ok=True)
    tmp_path = session_dir / f".{_MANIFEST_NAME}.{os.getpid()}.tmp"
    try:
        with open(tmp_path, "w", encoding="utf-8") as fh:
            json.dump(manifest, fh, indent=2, default=str)
        # os.replace is atomic on the same filesystem. If another process won
        # the race between the exists() check above and here, replace would
        # still clobber it -- so guard once more and only rename if still
        # absent; otherwise drop our temp file and let the first writer stand.
        if not manifest_path.exists():
            os.replace(tmp_path, manifest_path)
        else:
            tmp_path.unlink(missing_ok=True)
    finally:
        if tmp_path.exists():
            tmp_path.unlink(missing_ok=True)
    return manifest_path


__all__ = [
    "repo_root",
    "source_registry_path",
    "session_root",
    "read_pointer",
    "write_pointer",
    "resolve_session_dir",
    "active_session_dir",
    "ensure_session_dir",
    "event_log_default",
    "monitor_log_default",
    "orchestrator_log_dir_default",
    "sha256_file",
    "git_commit",
    "write_manifest",
]
