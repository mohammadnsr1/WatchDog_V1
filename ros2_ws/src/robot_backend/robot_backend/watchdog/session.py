"""Per-session directory and manifest for the WatchDog cell record.

One cell bringup produces one session directory. Every process that writes a
persistent artifact -- the per-process event logs, the orchestrator log, the
rosbag2 recording, and the manifest -- roots it there so a whole run's evidence
sits under a single timestamped path and correlates by wall_ns.

The directory is selected by WATCHDOG_SESSION_DIR. When that is set (normally by
sim_moveit_bringup.launch.py, and exported into the shell that launches the
agents so their MCP servers inherit it), every process resolves to the same
path and shares one session. When it is unset, each process falls back to a
per-process timestamped default under ~/.watchdog/sessions/ -- correct for
standalone, single-process use where cross-process sharing is not needed.

Nothing here is on any hot path: it runs once at process start.
"""

from __future__ import annotations

import hashlib
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import json

_SESSION_ENV = "WATCHDOG_SESSION_DIR"
_MANIFEST_NAME = "manifest.json"


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def session_dir_from_env() -> Path | None:
    """Return the session dir named by WATCHDOG_SESSION_DIR, or None if unset.

    Does not create it -- resolution only. Callers that need the directory to
    exist use ensure_session_dir().
    """
    raw = os.environ.get(_SESSION_ENV)
    if not raw or not raw.strip():
        return None
    return Path(os.path.expanduser(raw.strip()))


def default_session_dir() -> Path:
    """Per-process default session dir (not created): ~/.watchdog/sessions/<UTC>."""
    return Path.home() / ".watchdog" / "sessions" / _utc_stamp()


def ensure_session_dir() -> Path:
    """Resolve the session dir and create it, tolerating a concurrent creator.

    Uses WATCHDOG_SESSION_DIR when set (the shared-session case) and a
    per-process timestamped default otherwise. exist_ok=True makes the race
    between processes that both start against the same shared path harmless.
    """
    session_dir = session_dir_from_env() or default_session_dir()
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir


def event_log_default(server_name: str) -> str:
    """Default events_<server_name>.jsonl path for a process.

    Inside the session dir when WATCHDOG_SESSION_DIR is set, else the legacy
    ~/.watchdog/ location. This is only the DEFAULT -- an explicitly set
    WATCHDOG_EVENT_LOG / WATCHDOG_MONITOR_LOG env var still wins at the call
    site, so existing tooling is unaffected. The per-server filename is what
    keeps the four processes from colliding on one shared session path (a
    single shared WATCHDOG_EVENT_LOG would make them all write one file).
    """
    filename = f"events_{server_name}.jsonl"
    session_dir = session_dir_from_env()
    base = session_dir if session_dir is not None else (Path.home() / ".watchdog")
    return str(base / filename)


def monitor_log_default() -> str:
    """Default monitor_events.jsonl path for the WatchDog server."""
    session_dir = session_dir_from_env()
    base = session_dir if session_dir is not None else (Path.home() / ".watchdog")
    return str(base / "monitor_events.jsonl")


def orchestrator_log_dir_default() -> str:
    """Default orchestrator log directory.

    <session>/orchestrator when WATCHDOG_SESSION_DIR is set, else the legacy
    ~/.watchdog/orchestrator.
    """
    session_dir = session_dir_from_env()
    if session_dir is not None:
        return str(session_dir / "orchestrator")
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
    """
    manifest_path = session_dir / _MANIFEST_NAME
    if manifest_path.exists():
        return manifest_path

    session_id = session_dir.name
    manifest = {
        "session_id": session_id,
        "started_wall_ns": int(started_wall_ns),
        "registry": registry,
        "registry_sha256": registry_sha256,
        "registry_path": str(registry_path) if registry_path is not None else None,
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
    "session_dir_from_env",
    "default_session_dir",
    "ensure_session_dir",
    "event_log_default",
    "monitor_log_default",
    "orchestrator_log_dir_default",
    "sha256_file",
    "git_commit",
    "write_manifest",
]
