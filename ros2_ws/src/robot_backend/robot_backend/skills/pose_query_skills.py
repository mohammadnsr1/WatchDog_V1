"""Executor pose-query skills: TF lookup against the model registry.

Skills here take (tf_buffer, registry) rather than RobotBackend because their
only dependencies are the TF graph and registry metadata — not motion, gripper,
or scene manipulation. The MCP server creates and owns the TF buffer; these
functions are stateless wrappers that do the lookup and format the result.

BOUNDARY NOTE: these skills are for the EXECUTOR to locate objects before
planning a pick. They must NOT be used as the WatchDog's evidence source.
A supervisor would set up its own independent TF observation channel rather
than piggybacking on the executor's query path.
"""

from __future__ import annotations

from typing import Any

try:
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException
except ImportError:
    # Stubs so this module can be imported in pure-Python test environments
    # (e.g. when ROS shared libraries are absent). At runtime a real rclpy/tf2_ros
    # environment is required; the stubs only satisfy the import machinery.
    class Duration:  # type: ignore[no-redef]
        def __init__(self, *, seconds: float = 0.0, nanoseconds: int = 0) -> None:
            pass
    class Time:  # type: ignore[no-redef]
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass
    class Buffer:  # type: ignore[no-redef]
        pass
    class TransformException(Exception):  # type: ignore[no-redef]
        pass


def get_object_pose(
    tf_buffer: Buffer,
    registry: dict[str, Any],
    object_id: str,
    *,
    tf_timeout_sec: float = 1.0,
    clock=None,
) -> dict[str, Any]:
    """Return the current TF pose of one registry object in its lookup frame.

    Reads tf_frame and lookup_frame from the registry entry, then queries
    the provided TF buffer. TF is the single source of pose truth; this
    function does not touch MoveIt's planning scene.

    Parameters
    ----------
    tf_buffer:
        Active tf2_ros.Buffer (must have a TransformListener attached and
        be receiving transforms from the ROS network).
    registry:
        Loaded model_registry.yaml content (top-level dict with "objects" key).
    object_id:
        Registry key of the object (e.g. "red_cube").
    tf_timeout_sec:
        How long to wait for a transform to become available.
    clock:
        rclpy Clock (or any object with a .now() method returning rclpy.Time).
        When provided, the lookup requests the transform at clock.now(), which
        forces TF to wait for a fresh message rather than returning a cached
        value from a previous session. When None, falls back to Time() (latest
        cached — may be stale across process restarts or sim resets).

    Returns
    -------
    dict with keys:
        ok, command, object_id, frame_id, tf_frame, pose
        On failure: ok=False, error contains the reason.
    """
    objects = registry.get("objects", {})
    entry = objects.get(object_id)
    if not isinstance(entry, dict):
        return {
            "ok": False,
            "command": "get_object_pose",
            "object_id": object_id,
            "error": f"Object {object_id!r} not found in registry.",
        }

    tf_frame: str = str(entry.get("tf_frame", "")).strip()
    lookup_frame: str = str(entry.get("lookup_frame", "world")).strip() or "world"

    if not tf_frame:
        return {
            "ok": False,
            "command": "get_object_pose",
            "object_id": object_id,
            "error": f"Registry entry for {object_id!r} is missing tf_frame.",
        }

    lookup_time = clock.now() if clock is not None else Time()
    try:
        transform = tf_buffer.lookup_transform(
            lookup_frame,
            tf_frame,
            lookup_time,
            timeout=Duration(seconds=tf_timeout_sec),
        )
    except TransformException as exc:
        return {
            "ok": False,
            "command": "get_object_pose",
            "object_id": object_id,
            "error": f"TF lookup failed ({lookup_frame} ← {tf_frame}): {exc}",
        }

    t = transform.transform.translation
    r = transform.transform.rotation
    return {
        "ok": True,
        "command": "get_object_pose",
        "object_id": object_id,
        "frame_id": lookup_frame,
        "tf_frame": tf_frame,
        "pose": {
            "position": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
            "orientation": {
                "x": float(r.x),
                "y": float(r.y),
                "z": float(r.z),
                "w": float(r.w),
            },
        },
    }


def get_object_poses(
    tf_buffer: Buffer,
    registry: dict[str, Any],
    *,
    enabled_only: bool = True,
    graspable_only: bool = False,
    tf_timeout_sec: float = 1.0,
    clock=None,
) -> dict[str, Any]:
    """Return TF poses for all matching registry objects.

    Iterates the registry and calls get_object_pose for each matching entry.
    Failures are collected rather than raised so one bad TF frame does not
    suppress results for the remaining objects.

    Parameters
    ----------
    enabled_only:
        Skip entries with enabled: false (default: True).
    graspable_only:
        If True, only return objects with graspable: true in the registry.
    tf_timeout_sec:
        Per-object TF lookup timeout.
    clock:
        Passed through to get_object_pose. See that function for details.
    """
    objects = registry.get("objects", {})
    poses: list[dict[str, Any]] = []
    failed: list[dict[str, Any]] = []

    for object_id, entry in objects.items():
        if not isinstance(entry, dict):
            continue
        if enabled_only and not bool(entry.get("enabled", True)):
            continue
        if graspable_only and not bool(entry.get("graspable", False)):
            continue

        result = get_object_pose(
            tf_buffer, registry, object_id, tf_timeout_sec=tf_timeout_sec, clock=clock
        )
        if result["ok"]:
            poses.append(result)
        else:
            failed.append({"object_id": object_id, "error": result.get("error", "")})

    return {
        "ok": len(failed) == 0,
        "command": "get_object_poses",
        "poses": poses,
        "failed": failed,
    }
