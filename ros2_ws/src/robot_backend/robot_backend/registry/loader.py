"""Single ROS-free parser and validator for model_registry.yaml.

Relative asset paths use the registry file's directory by default. A top-level
``asset_root`` may override it; a relative override is itself resolved against
the registry file's directory.
"""

from __future__ import annotations

import json
import os
import warnings
from pathlib import Path
from typing import Any

import yaml

from robot_backend.utils.geometry import make_pose

from .model import (
    CollisionSpec,
    CollisionVariant,
    MeshCollisionSpec,
    RegisteredObject,
    Registry,
    Vector3Tuple,
)


# Class 4 (registry corruption): the EXECUTOR-side registry override.
#
# A Class 4 run hands the executor a corrupted registry while the physical
# cell (scene_tf_sync, MoveIt) and, in S1-S3, the WatchDog keep the clean one.
# The override is an env var rather than a config edit so a batch can flip it
# per run (`EXECUTOR_REGISTRY_PATH=... ./run_executor.sh ...`): the executor's
# MCP servers inherit it from the launching shell.
#
# It is honored ONLY in an executor-role process — the same env-var role test
# role_context.detect_role() uses (EXECUTOR_TASK_PATH set, WATCHDOG_TASK_PATH
# unset). That guard is load-bearing: the watchdog session's servers are also
# launched from a shell, so a stray export would otherwise corrupt the
# watchdog's copy too and the independence comparison with it. A non-executor
# process that sees the var warns and loads the normal registry. The cell-side
# scene_tf_sync node does not call this at all: physics and planning always
# run on the real registry.
REGISTRY_OVERRIDE_ENV = "EXECUTOR_REGISTRY_PATH"


def executor_registry_override() -> Path | None:
    """The overriding registry path for THIS process, or None.

    Raises when the override names a missing file — a Class 4 run silently
    falling back to the clean registry would score as "no corruption" and
    poison the batch, so the launch must die instead.

    When the override applies, its provenance (path + sha256) is written to
    <session>/registry_provenance_executor.json so the scorer can see which
    registry the executor actually read without trusting the executor's own
    bookkeeping. Best-effort: a provenance write failure never blocks a load.
    """
    raw = os.environ.get(REGISTRY_OVERRIDE_ENV, "").strip()
    if not raw:
        return None
    is_executor = (
        os.environ.get("EXECUTOR_TASK_PATH", "").strip()
        and not os.environ.get("WATCHDOG_TASK_PATH", "").strip()
    )
    if not is_executor:
        warnings.warn(
            f"{REGISTRY_OVERRIDE_ENV} is set but this is not an executor-role "
            f"process; ignoring it and loading the normal registry. The "
            f"override corrupts the EXECUTOR's view only, by design.",
            stacklevel=2,
        )
        return None
    path = Path(os.path.expanduser(raw)).resolve()
    if not path.is_file():
        raise FileNotFoundError(
            f"{REGISTRY_OVERRIDE_ENV}={raw!r} does not name a file. Refusing "
            f"to fall back to the clean registry: a Class 4 run that silently "
            f"loaded the clean copy would score as if no corruption happened."
        )
    _record_override_provenance(path)
    return path


def _record_override_provenance(path: Path) -> None:
    try:
        from robot_backend.watchdog import session as session_helper

        session_dir = session_helper.active_session_dir()
        if session_dir is None:
            return
        record = {
            "role": "executor",
            "registry_path": str(path),
            "registry_sha256": session_helper.sha256_file(path),
            "override_env": REGISTRY_OVERRIDE_ENV,
        }
        out = session_dir / "registry_provenance_executor.json"
        out.write_text(json.dumps(record, indent=2), encoding="utf-8")
    except Exception:  # pragma: no cover - provenance must never block a load
        pass


def load_registry(path: Path | str) -> Registry:
    """Parse and validate one registry, raising with object_id on any bad object."""
    resolved_path = Path(path).expanduser().resolve()
    if not resolved_path.is_file():
        raise FileNotFoundError(f"Model registry not found: {resolved_path}")

    raw_text = resolved_path.read_bytes().decode("utf-8")
    raw_dict = yaml.safe_load(raw_text) or {}
    if not isinstance(raw_dict, dict):
        raise ValueError("model_registry.yaml must be a top-level mapping.")

    asset_root = _resolve_asset_root(raw_dict.get("asset_root"), resolved_path.parent)
    raw_objects = raw_dict.get("objects", {})
    if not isinstance(raw_objects, dict):
        raise ValueError("model_registry.yaml objects section must be a mapping.")

    objects: dict[str, RegisteredObject] = {}
    for registry_key, raw_object in raw_objects.items():
        objects[str(registry_key)] = _parse_object(
            str(registry_key), raw_object, asset_root
        )

    return Registry(
        raw_text=raw_text,
        raw_dict=raw_dict,
        asset_root=asset_root,
        objects=objects,
    )


def _resolve_asset_root(value: Any, registry_dir: Path) -> Path:
    if value is None:
        return registry_dir
    if not isinstance(value, str) or not value.strip():
        raise ValueError("asset_root must be a non-empty path string when present.")
    root = Path(value).expanduser()
    return (root if root.is_absolute() else registry_dir / root).resolve()


def _parse_object(
    registry_key: str,
    value: Any,
    asset_root: Path,
) -> RegisteredObject:
    error_id = registry_key
    if not isinstance(value, dict):
        raise ValueError(f"Object {error_id!r}: registry entry must be a mapping.")

    authored_id = value.get("object_id")
    if isinstance(authored_id, str) and authored_id.strip():
        error_id = authored_id.strip()

    object_id = _required_string(value, "object_id", error_id)
    tf_frame = _required_string(value, "tf_frame", error_id)
    lookup_frame = _required_string(value, "lookup_frame", error_id)
    class_name = _required_string(value, "class_name", error_id)
    source = _required_string(value, "source", error_id)
    graspable = _required_bool(value, "graspable", error_id)
    enabled = _required_bool(value, "enabled", error_id)
    kind = _parse_kind(value, error_id)
    if kind == "frame" and graspable:
        raise ValueError(
            f"Object {error_id!r}: kind 'frame' denotes a pose-only landmark "
            "with no physical body, so graspable must be false."
        )

    raw_spec = value.get("spec", {})
    if not isinstance(raw_spec, dict):
        raise ValueError(f"Object {error_id!r}: spec must be a mapping when present.")

    raw_collision = value.get("collision")
    if not isinstance(raw_collision, dict):
        raise ValueError(f"Object {error_id!r}: collision must be a mapping.")
    collision = _parse_collision(raw_collision, error_id, asset_root)
    mesh_pose_required = isinstance(collision, MeshCollisionSpec)
    if mesh_pose_required and "pose_offset" not in raw_collision:
        raise ValueError(
            f"Object {error_id!r}: mesh collision requires an explicitly authored "
            "collision.pose_offset."
        )
    pose_offset = _parse_pose_offset(
        raw_collision.get("pose_offset"),
        error_id,
        lookup_frame,
        require_complete=mesh_pose_required,
    )

    return RegisteredObject(
        object_id=object_id,
        tf_frame=tf_frame,
        lookup_frame=lookup_frame,
        class_name=class_name,
        source=source,
        graspable=graspable,
        enabled=enabled,
        kind=kind,
        spec=raw_spec,
        collision=collision,
        pose_offset=pose_offset,
    )


def _parse_collision(
    value: dict[str, Any],
    object_id: str,
    asset_root: Path,
) -> CollisionVariant:
    shape_type = value.get("type")
    if not isinstance(shape_type, str) or not shape_type.strip():
        raise ValueError(f"Object {object_id!r}: collision.type must be a non-empty string.")
    shape_type = shape_type.strip().lower()
    if shape_type == "box":
        if "mesh_path" in value:
            raise ValueError(
                f"Object {object_id!r}: box collision forbids collision.mesh_path."
            )
        dimensions = _parse_positive_vector(
            value.get("dimensions"),
            object_id,
            "collision.dimensions",
            required_message="box collision requires dimensions [x, y, z]",
        )
        return CollisionSpec(type="box", dimensions=dimensions)

    if shape_type == "mesh":
        if "dimensions" in value:
            raise ValueError(
                f"Object {object_id!r}: mesh collision forbids collision.dimensions."
            )
        authored_path = value.get("mesh_path")
        if not isinstance(authored_path, str) or not authored_path.strip():
            raise ValueError(
                f"Object {object_id!r}: mesh collision requires collision.mesh_path."
            )
        authored_path = authored_path.strip()
        relative_path = Path(authored_path)
        if relative_path.is_absolute():
            raise ValueError(
                f"Object {object_id!r}: collision.mesh_path must be relative to asset_root."
            )
        resolved_path = (asset_root / relative_path).resolve()
        try:
            resolved_path.relative_to(asset_root)
        except ValueError as exc:
            raise ValueError(
                f"Object {object_id!r}: collision.mesh_path escapes asset_root: "
                f"{authored_path!r}."
            ) from exc
        if not resolved_path.is_file():
            raise ValueError(
                f"Object {object_id!r}: collision.mesh_path does not exist: "
                f"{resolved_path}."
            )
        scale = _parse_positive_vector(
            value.get("scale", [1.0, 1.0, 1.0]),
            object_id,
            "collision.scale",
            required_message="collision.scale must be [x, y, z]",
        )
        return MeshCollisionSpec(
            type="mesh",
            mesh_path=authored_path,
            resolved_mesh_path=resolved_path,
            scale=scale,
        )

    raise ValueError(
        f"Object {object_id!r}: unsupported collision.type {shape_type!r}; "
        "known types: box, mesh."
    )


def _parse_positive_vector(
    value: Any,
    object_id: str,
    field: str,
    *,
    required_message: str,
) -> Vector3Tuple:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"Object {object_id!r}: {required_message}.")
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Object {object_id!r}: {field} must contain numbers.") from exc
    if any(component <= 0.0 for component in result):
        raise ValueError(f"Object {object_id!r}: {field} must all be positive.")
    return result  # type: ignore[return-value]


def _parse_pose_offset(
    value: Any,
    object_id: str,
    lookup_frame: str,
    *,
    require_complete: bool = False,
) -> dict[str, Any]:
    if value is None:
        if require_complete:
            raise ValueError(
                f"Object {object_id!r}: mesh collision.pose_offset must be a mapping."
            )
        return make_pose(
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            frame_id=lookup_frame,
        )
    if not isinstance(value, dict):
        raise ValueError(f"Object {object_id!r}: collision.pose_offset must be a mapping.")
    position = value.get("position", {})
    orientation = value.get("orientation", {})
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(
            f"Object {object_id!r}: collision.pose_offset position/orientation must be mappings."
        )
    if require_complete:
        missing_position = [axis for axis in ("x", "y", "z") if axis not in position]
        missing_orientation = [
            axis for axis in ("x", "y", "z", "w") if axis not in orientation
        ]
        if missing_position or missing_orientation:
            raise ValueError(
                f"Object {object_id!r}: mesh collision.pose_offset must explicitly "
                "author position x/y/z and orientation x/y/z/w."
            )
    try:
        return make_pose(
            {
                "x": float(position.get("x", 0.0)),
                "y": float(position.get("y", 0.0)),
                "z": float(position.get("z", 0.0)),
            },
            {
                "x": float(orientation.get("x", 0.0)),
                "y": float(orientation.get("y", 0.0)),
                "z": float(orientation.get("z", 0.0)),
                "w": float(orientation.get("w", 1.0)),
            },
            frame_id=lookup_frame,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"Object {object_id!r}: invalid collision.pose_offset: {exc}"
        ) from exc


def _required_string(value: dict[str, Any], field: str, object_id: str) -> str:
    item = value.get(field)
    if not isinstance(item, str) or not item.strip():
        raise ValueError(f"Object {object_id!r}: required field {field!r} is missing or empty.")
    return item.strip()


def _required_bool(value: dict[str, Any], field: str, object_id: str) -> bool:
    item = value.get(field)
    if not isinstance(item, bool):
        raise ValueError(f"Object {object_id!r}: required field {field!r} must be boolean.")
    return item


_VALID_KINDS = ("object", "frame")


def _parse_kind(value: dict[str, Any], object_id: str) -> str:
    """Parse `kind`, defaulting to "object" when the entry does not author it.

    Defaulted rather than required so a registry written before `kind` existed
    still loads with its previous meaning: every entry was treated as a body.
    """
    item = value.get("kind", "object")
    if not isinstance(item, str) or item.strip().lower() not in _VALID_KINDS:
        raise ValueError(
            f"Object {object_id!r}: kind must be one of {list(_VALID_KINDS)} "
            f"when present, got {item!r}."
        )
    return item.strip().lower()
