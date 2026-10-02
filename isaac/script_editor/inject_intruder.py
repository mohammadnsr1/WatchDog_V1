"""Operator tool: inject intruder objects into the live Isaac scene.

USAGE (paste into Isaac Sim's Script Editor while the cell is running):

    import sys
    sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1/isaac/script_editor")
    import inject_intruder as ii

    # Immediately drop a red 5 cm cube on the table:
    ii.inject("intruder_a", shape="cube", size_m=0.05,
              position_xyz_world=(0.4, -0.2, 0.76), color_rgb=(0.9, 0.1, 0.1),
              physics=True, label="red cube near base2",
              expected_impact="benign")

    # Inject when the executor begins a step (fires on begin_step in
    # events_scene.jsonl), or after a delay:
    ii.inject("intruder_b", shape="sphere", size_m=0.04,
              position_xyz_world=(0.3, 0.1, 0.80), color_rgb=(0.1, 0.1, 0.9),
              physics=False, label="blue sphere over tray",
              expected_impact="violates", at_step="place_u_green")
    ii.move("intruder_a", (0.5, -0.1, 0.76), delay_s=10.0)
    ii.remove("intruder_b", at_step="place_hex_red")

    ii.list_injections()   # executed actions (ground truth) + pending triggers

Every EXECUTED action appends one JSON line to
<session_dir>/ground_truth_injections.jsonl, where <session_dir> is resolved
live from ~/.watchdog/current_session. That file is the run's ground truth for
observation scoring; this script never publishes to /watchdog/events and never
touches ROS -- the agents must discover intruders through the camera alone.

Prims are created under /World/WatchdogIntruders. Step 0 of the Stage 3 audit
confirmed the scene's object TF publisher
(/Graph/Objects_TF/PublisherTF, a ROS2PublishTransformTree saved in
assembly_room.usd) publishes an EXPLICIT targetPrims list -- its parentPrim
/World only names the reference frame -- so prims under this root get no TF
frame unless someone adds them to that list. Do not add them.

Intruder names are checked against the model registry: a name that collides
with any registry object id or tf_frame (including disabled entries such as
red_cube -- get_object_pose ignores `enabled` and would resolve them) or with
a cell frame is refused.

The module splits into a pure-Python core (unit-testable outside Isaac; see
test_inject_intruder.py) and an Isaac layer whose omni.usd / pxr /
omni.kit.app imports happen lazily inside functions. at_step / delay_s
scheduling runs on an omni.kit.app update subscription and only ever reads
files -- Isaac's main thread is never blocked.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

# ---------------------------------------------------------------------------
# Pure core -- no Isaac imports anywhere below until the "Isaac layer" marker.
# ---------------------------------------------------------------------------

INTRUDER_ROOT = "/World/WatchdogIntruders"
SHAPES = ("cube", "cylinder", "sphere")
EXPECTED_IMPACTS = ("benign", "violates")
GROUND_TRUTH_FILENAME = "ground_truth_injections.jsonl"

_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

_REPO_ROOT = Path(__file__).resolve().parents[2]
_REGISTRY_PATH = (
    _REPO_ROOT / "ros2_ws" / "src" / "robot_backend" / "config" / "model_registry.yaml"
)
_POINTER_FILE = Path.home() / ".watchdog" / "current_session"

# Names that are TF frames or reference frames regardless of the registry.
_BUILTIN_FORBIDDEN = {"world", "World", "WatchdogIntruders"}


def script_sha256() -> str:
    """sha256 of this script file, recorded on every ground-truth line."""
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def load_forbidden_names(registry_path: Path | None = None) -> set[str]:
    """Every name an intruder must never use: registry object ids, tf_frame
    and lookup_frame values (DISABLED entries included -- red_cube still
    resolves through get_object_pose), cell frames, and builtins."""
    import yaml

    path = registry_path or _REGISTRY_PATH
    with open(path, encoding="utf-8") as handle:
        registry = yaml.safe_load(handle) or {}
    forbidden = set(_BUILTIN_FORBIDDEN)
    objects = registry.get("objects") or {}
    if isinstance(objects, dict):
        for object_id, entry in objects.items():
            forbidden.add(str(object_id))
            if isinstance(entry, dict):
                for key in ("tf_frame", "lookup_frame"):
                    value = entry.get(key)
                    if isinstance(value, str) and value:
                        forbidden.add(value)
    cell = registry.get("cell") or {}
    if isinstance(cell, dict):
        base = cell.get("base_frame")
        if isinstance(base, str) and base:
            forbidden.add(base)
        gripper = cell.get("gripper") or {}
        if isinstance(gripper, dict):
            for key, value in gripper.items():
                if key.endswith("_frame") and isinstance(value, str) and value:
                    forbidden.add(value)
    return forbidden


def validate_name(name: str, forbidden: set[str]) -> str:
    """Return the validated intruder name, or raise ValueError."""
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise ValueError(
            f"intruder name {name!r} is not a valid prim name "
            "(letters, digits, underscores; must not start with a digit)."
        )
    if name in forbidden:
        raise ValueError(
            f"intruder name {name!r} collides with a registry object id, a "
            "TF frame, or a cell frame -- an intruder must never reuse one "
            "(disabled entries such as red_cube still resolve through "
            "get_object_pose). Pick a name that appears nowhere in "
            "model_registry.yaml."
        )
    return name


def validate_inject_args(
    shape: str,
    size_m: float,
    position_xyz_world: Any,
    color_rgb: Any,
    expected_impact: str,
) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
    """Validate and normalise inject()'s geometry arguments."""
    if shape not in SHAPES:
        raise ValueError(f"shape must be one of {list(SHAPES)}, got {shape!r}.")
    size = float(size_m)
    if not (size > 0):
        raise ValueError(f"size_m must be > 0, got {size_m!r}.")
    position = _xyz(position_xyz_world, "position_xyz_world")
    color = _xyz(color_rgb, "color_rgb")
    if any(not (0.0 <= c <= 1.0) for c in color):
        raise ValueError(f"color_rgb components must be in [0, 1], got {color_rgb!r}.")
    if expected_impact not in EXPECTED_IMPACTS:
        raise ValueError(
            f"expected_impact must be one of {list(EXPECTED_IMPACTS)}, "
            f"got {expected_impact!r}."
        )
    return size, position, color


def _xyz(value: Any, label: str) -> tuple[float, float, float]:
    try:
        x, y, z = (float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be three numbers, got {value!r}.") from exc
    return (x, y, z)


# ---------------------------------------------------------------------------
# Camera projection: where a world point lands in the depth/RGB image, so the
# ground truth can say whether an injected intruder was even in view.
#
# Extrinsics are the static world -> camera_depth_optical_frame transform from
# ros2_ws/src/sim_moveit_bringup/config/cameras.yaml (the camera never moves).
# Intrinsics were read off /camera/depth/camera_info on 2026-09-30:
# 640x360, fx = fy = 554.2563, cx = 320, cy = 180, no distortion.
# ---------------------------------------------------------------------------

CAMERA_EXTRINSICS = {
    "translation_world": (0.25, -0.2, 1.7),
    "rotation_xyzw": (0.0, 0.99863, -0.05234, 0.0),
}
CAMERA_INTRINSICS = {
    "width": 640, "height": 360,
    "fx": 554.2563, "fy": 554.2563, "cx": 320.0, "cy": 180.0,
}
# An intruder whose projected centre is within this many pixels of the image
# border is treated as not reliably in view.
CAMERA_VIEW_MARGIN_PX = 20.0


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def project_world_point(
    point_xyz_world: Any,
) -> tuple[tuple[float, float] | None, float]:
    """((u, v) pixel or None if behind the camera, depth along the optical axis)."""
    px, py, pz = _xyz(point_xyz_world, "point_xyz_world")
    tx, ty, tz = CAMERA_EXTRINSICS["translation_world"]
    rotation = _quat_to_matrix(*CAMERA_EXTRINSICS["rotation_xyzw"])
    d = (px - tx, py - ty, pz - tz)
    # p_cam = R^T (p_world - t): the rotation columns are the camera axes in
    # world, so the transpose maps a world vector into the optical frame
    # (+Z forward, +X right, +Y down).
    cam = [sum(rotation[r][c] * d[r] for r in range(3)) for c in range(3)]
    if cam[2] <= 0:
        return None, cam[2]
    k = CAMERA_INTRINSICS
    u = k["fx"] * cam[0] / cam[2] + k["cx"]
    v = k["fy"] * cam[1] / cam[2] + k["cy"]
    return (u, v), cam[2]


def camera_view_fields(point_xyz_world: Any) -> dict[str, Any]:
    """The three ground-truth fields describing a point's visibility.

    pixel_uv is where the point's CENTRE projects ([u, v], rounded to 0.1 px;
    null if behind the camera). edge_margin_px is the centre's distance to the
    nearest image border (negative = outside the image). in_camera_view is
    true when the centre is inside the image with at least
    CAMERA_VIEW_MARGIN_PX of margin -- centre-only, so a large object whose
    centre sits just inside the margin may still be partly cropped.
    """
    pixel, depth = project_world_point(point_xyz_world)
    if pixel is None:
        return {"pixel_uv": None, "in_camera_view": False, "edge_margin_px": None}
    u, v = pixel
    k = CAMERA_INTRINSICS
    margin = min(u, v, k["width"] - u, k["height"] - v)
    return {
        "pixel_uv": [round(u, 1), round(v, 1)],
        "in_camera_view": bool(depth > 0 and margin >= CAMERA_VIEW_MARGIN_PX),
        "edge_margin_px": round(margin, 1),
    }


def resolve_session_dir(pointer_file: Path | None = None) -> Path:
    """The live session dir from ~/.watchdog/current_session, or raise."""
    pointer = pointer_file or _POINTER_FILE
    try:
        raw = pointer.read_text(encoding="utf-8").strip()
    except OSError as exc:
        raise RuntimeError(
            f"cannot read {pointer}: {exc}. Is the cell running? The bringup "
            "writes this pointer at session start."
        ) from exc
    if not raw:
        raise RuntimeError(f"{pointer} is empty; no active session.")
    session_dir = Path(os.path.expanduser(raw))
    if not session_dir.is_dir():
        raise RuntimeError(
            f"{pointer} names {session_dir}, which is not a directory."
        )
    return session_dir


class StepTriggerMatcher:
    """Incrementally match begin_step events for one step_id in a JSONL tail.

    feed() takes raw text (any number of lines, possibly ending mid-line --
    exactly what an incremental file read yields), buffers the partial tail,
    and returns True once a complete line parses as an EventEnvelope whose
    event_type is begin_step and whose payload.step_id matches. Malformed
    lines are skipped: a corrupt line must not stop the trigger from seeing a
    later good one.
    """

    def __init__(self, step_id: str) -> None:
        if not isinstance(step_id, str) or not step_id:
            raise ValueError("step_id must be a non-empty string.")
        self.step_id = step_id
        self._partial = ""
        self.matched = False

    def feed(self, text: str) -> bool:
        if self.matched:
            return True
        self._partial += text
        lines = self._partial.split("\n")
        # The piece after the last newline is an incomplete line; keep it.
        self._partial = lines.pop()
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if not isinstance(event, dict) or event.get("event_type") != "begin_step":
                continue
            payload = event.get("payload")
            if isinstance(payload, dict) and payload.get("step_id") == self.step_id:
                self.matched = True
                return True
        return False


def build_ground_truth_record(
    *,
    action: str,
    name: str,
    prim_path: str,
    trigger: str,
    wall_ns: int,
    session_dir: str,
    shape: str | None = None,
    size_m: float | None = None,
    position_xyz_world: tuple[float, float, float] | None = None,
    yaw_deg: float | None = None,
    color_rgb: tuple[float, float, float] | None = None,
    physics: bool | None = None,
    label: str | None = None,
    expected_impact: str | None = None,
    notes: str = "",
    sha256: str | None = None,
) -> dict[str, Any]:
    """One ground-truth line, every key always present (null when n/a).

    inject and move records (the ones carrying a position) also carry the
    camera-visibility triple from camera_view_fields, so scoring can tell an
    intruder the camera could see from one it never could.
    """
    visibility = (
        camera_view_fields(position_xyz_world)
        if position_xyz_world is not None
        else {"pixel_uv": None, "in_camera_view": None, "edge_margin_px": None}
    )
    return {
        **visibility,
        "action": action,
        "name": name,
        "prim_path": prim_path,
        "shape": shape,
        "size_m": size_m,
        "position_xyz_world": list(position_xyz_world) if position_xyz_world else None,
        "yaw_deg": yaw_deg,
        "color_rgb": list(color_rgb) if color_rgb else None,
        "physics": physics,
        "label": label,
        "expected_impact": expected_impact,
        "notes": notes,
        "trigger": trigger,
        "wall_ns": int(wall_ns),
        "wall_time_iso": datetime.fromtimestamp(
            wall_ns / 1e9, tz=timezone.utc
        ).isoformat(),
        "session_dir": str(session_dir),
        "script_sha256": sha256 if sha256 is not None else script_sha256(),
    }


def append_jsonl(path: Path, record: dict[str, Any]) -> None:
    """Append one JSON line: open-append + flush + fsync, so a line that was
    written survives whatever happens to Isaac afterwards."""
    path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(record, separators=(",", ":")) + "\n"
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(line)
        handle.flush()
        os.fsync(handle.fileno())


def trigger_text(at_step: str | None, delay_s: float) -> str:
    """The trigger field for a ground-truth record."""
    if at_step is not None:
        return f"at_step:{at_step}"
    if delay_s and delay_s > 0:
        return f"delay:{delay_s:g}"
    return "immediate"


# ---------------------------------------------------------------------------
# Isaac layer. Everything from here down imports omni.* / pxr lazily so the
# module (and the pure core above) imports cleanly outside Isaac.
# ---------------------------------------------------------------------------

_pending: list[dict[str, Any]] = []
_executed: list[dict[str, Any]] = []
_update_subscription: Any = None
_forbidden_cache: set[str] | None = None


def _forbidden() -> set[str]:
    global _forbidden_cache
    if _forbidden_cache is None:
        _forbidden_cache = load_forbidden_names()
    return _forbidden_cache


def _stage() -> Any:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no USD stage is open in this Isaac instance.")
    return stage


def _ensure_root(stage: Any) -> None:
    from pxr import UsdGeom

    if not stage.GetPrimAtPath(INTRUDER_ROOT):
        UsdGeom.Xform.Define(stage, INTRUDER_ROOT)


def _prim_path(name: str) -> str:
    return f"{INTRUDER_ROOT}/{name}"


def _apply_inject(
    name: str,
    shape: str,
    size: float,
    position: tuple[float, float, float],
    yaw_deg: float,
    color: tuple[float, float, float],
    physics: bool,
) -> str:
    """Create the prim. Returns the prim path. Isaac main thread only."""
    from pxr import Gf, UsdGeom, UsdPhysics

    stage = _stage()
    _ensure_root(stage)
    path = _prim_path(name)
    if stage.GetPrimAtPath(path):
        raise ValueError(f"{path} already exists; use move() or remove().")

    if shape == "cube":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.GetSizeAttr().Set(float(size))
    elif shape == "cylinder":
        geom = UsdGeom.Cylinder.Define(stage, path)
        geom.GetRadiusAttr().Set(float(size) / 2.0)
        geom.GetHeightAttr().Set(float(size))
    else:  # sphere
        geom = UsdGeom.Sphere.Define(stage, path)
        geom.GetRadiusAttr().Set(float(size) / 2.0)

    xform = UsdGeom.Xformable(geom.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddRotateZOp().Set(float(yaw_deg))
    UsdGeom.Gprim(geom.GetPrim()).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

    if physics:
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(geom.GetPrim())
    return path


def _apply_move(
    name: str, position: tuple[float, float, float], yaw_deg: float | None
) -> str:
    from pxr import Gf, UsdGeom

    stage = _stage()
    path = _prim_path(name)
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise ValueError(f"{path} does not exist; inject() it first.")
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        op_type = op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*position))
        elif op_type == UsdGeom.XformOp.TypeRotateZ and yaw_deg is not None:
            op.Set(float(yaw_deg))
    return path


def _apply_remove(name: str) -> str:
    stage = _stage()
    path = _prim_path(name)
    if not stage.GetPrimAtPath(path):
        raise ValueError(f"{path} does not exist.")
    stage.RemovePrim(path)
    return path


def _record(record: dict[str, Any]) -> dict[str, Any]:
    session_dir = Path(record["session_dir"])
    append_jsonl(session_dir / GROUND_TRUTH_FILENAME, record)
    _executed.append(record)
    print(f"[inject_intruder] {record['action']} {record['name']} "
          f"({record['trigger']}) -> {session_dir / GROUND_TRUTH_FILENAME}")
    return record


def _schedule(
    description: str,
    at_step: str | None,
    delay_s: float,
    apply_fn: Callable[[], dict[str, Any]],
) -> dict[str, Any]:
    """Run now, or queue on the kit update stream. Never blocks the main thread."""
    if at_step is None and (not delay_s or delay_s <= 0):
        return apply_fn()

    entry: dict[str, Any] = {
        "description": description,
        "trigger": trigger_text(at_step, delay_s),
        "apply": apply_fn,
        "deadline_monotonic": (
            time.monotonic() + float(delay_s) if at_step is None else None
        ),
        "matcher": StepTriggerMatcher(at_step) if at_step is not None else None,
        # Tail state for events_scene.jsonl: resolved lazily on the first
        # tick so a trigger armed before the session exists still works.
        "events_path": None,
        "offset": 0,
    }
    _pending.append(entry)
    _ensure_update_subscription()
    print(f"[inject_intruder] queued: {description} ({entry['trigger']})")
    return entry


def _ensure_update_subscription() -> None:
    global _update_subscription
    if _update_subscription is not None:
        return
    import omni.kit.app

    stream = omni.kit.app.get_app().get_update_event_stream()
    _update_subscription = stream.create_subscription_to_pop(
        _on_update, name="watchdog_intruder_scheduler"
    )


def _on_update(_event: Any = None) -> None:
    """One kit update tick: check every pending trigger; fire the due ones.

    File reads only (an incremental tail of events_scene.jsonl), bounded per
    tick, so this never blocks Isaac's main thread.
    """
    due: list[dict[str, Any]] = []
    for entry in list(_pending):
        try:
            if entry["matcher"] is not None:
                if entry["events_path"] is None:
                    try:
                        session_dir = resolve_session_dir()
                    except RuntimeError:
                        continue  # no session yet; keep waiting
                    entry["events_path"] = session_dir / "events_scene.jsonl"
                path = entry["events_path"]
                if not path.exists():
                    continue
                with open(path, "r", encoding="utf-8") as handle:
                    handle.seek(entry["offset"])
                    chunk = handle.read()
                    entry["offset"] = handle.tell()
                if chunk and entry["matcher"].feed(chunk):
                    due.append(entry)
            elif time.monotonic() >= entry["deadline_monotonic"]:
                due.append(entry)
        except Exception as exc:
            print(f"[inject_intruder] trigger check failed, kept pending: {exc}")
    for entry in due:
        _pending.remove(entry)
        try:
            entry["apply"]()
        except Exception as exc:
            print(f"[inject_intruder] queued action FAILED: "
                  f"{entry['description']}: {exc}")
    # The subscription is deliberately kept even when _pending drains:
    # releasing it from inside its own callback is not guaranteed safe in
    # kit, and an idle tick over an empty list costs nothing.


# ---------------------------------------------------------------------------
# Operator API
# ---------------------------------------------------------------------------

def inject(
    name: str,
    shape: str = "cube",
    size_m: float = 0.05,
    position_xyz_world: Any = (0.4, 0.0, 0.8),
    yaw_deg: float = 0.0,
    color_rgb: Any = (0.8, 0.1, 0.1),
    physics: bool = True,
    label: str = "",
    expected_impact: str = "benign",
    notes: str = "",
    at_step: str | None = None,
    delay_s: float = 0,
) -> dict[str, Any]:
    """Create an intruder prim under /World/WatchdogIntruders.

    Validates everything NOW (name collisions against the registry, shapes,
    colors), whatever the trigger, so a queued action cannot fail validation
    minutes later when nobody is watching. The ground-truth line is written
    at the moment the prim change is actually applied.
    """
    validate_name(name, _forbidden())
    size, position, color = validate_inject_args(
        shape, size_m, position_xyz_world, color_rgb, expected_impact
    )
    yaw = float(yaw_deg)
    trigger = trigger_text(at_step, delay_s)
    sha = script_sha256()
    visibility = camera_view_fields(position)
    if not visibility["in_camera_view"]:
        print(
            f"[inject_intruder] WARNING: {name} at {position} projects to "
            f"pixel {visibility['pixel_uv']} (edge margin "
            f"{visibility['edge_margin_px']} px) -- outside the camera's "
            f"reliable view. Proceeding anyway; the ground truth records it."
        )

    def apply() -> dict[str, Any]:
        session_dir = resolve_session_dir()
        path = _apply_inject(name, shape, size, position, yaw, color, bool(physics))
        return _record(build_ground_truth_record(
            action="inject", name=name, prim_path=path, trigger=trigger,
            wall_ns=time.time_ns(), session_dir=str(session_dir),
            shape=shape, size_m=size, position_xyz_world=position, yaw_deg=yaw,
            color_rgb=color, physics=bool(physics), label=label,
            expected_impact=expected_impact, notes=notes, sha256=sha,
        ))

    return _schedule(f"inject {name}", at_step, delay_s, apply)


def move(
    name: str,
    position_xyz_world: Any,
    yaw_deg: float | None = None,
    at_step: str | None = None,
    delay_s: float = 0,
) -> dict[str, Any]:
    """Teleport an existing intruder to a new world position (and yaw)."""
    validate_name(name, _forbidden())
    position = _xyz(position_xyz_world, "position_xyz_world")
    yaw = float(yaw_deg) if yaw_deg is not None else None
    trigger = trigger_text(at_step, delay_s)
    sha = script_sha256()

    def apply() -> dict[str, Any]:
        session_dir = resolve_session_dir()
        path = _apply_move(name, position, yaw)
        return _record(build_ground_truth_record(
            action="move", name=name, prim_path=path, trigger=trigger,
            wall_ns=time.time_ns(), session_dir=str(session_dir),
            position_xyz_world=position, yaw_deg=yaw, sha256=sha,
        ))

    return _schedule(f"move {name}", at_step, delay_s, apply)


def remove(
    name: str,
    at_step: str | None = None,
    delay_s: float = 0,
) -> dict[str, Any]:
    """Delete an intruder prim."""
    validate_name(name, _forbidden())
    trigger = trigger_text(at_step, delay_s)
    sha = script_sha256()

    def apply() -> dict[str, Any]:
        session_dir = resolve_session_dir()
        path = _apply_remove(name)
        return _record(build_ground_truth_record(
            action="remove", name=name, prim_path=path, trigger=trigger,
            wall_ns=time.time_ns(), session_dir=str(session_dir), sha256=sha,
        ))

    return _schedule(f"remove {name}", at_step, delay_s, apply)


def list_injections() -> dict[str, Any]:
    """Executed actions (from the session's ground truth file) and pending ones."""
    executed: list[dict[str, Any]] = []
    try:
        session_dir = resolve_session_dir()
        gt_path = session_dir / GROUND_TRUTH_FILENAME
        if gt_path.exists():
            for line in gt_path.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    executed.append(json.loads(line))
                except ValueError:
                    executed.append({"unparseable_line": line})
    except RuntimeError as exc:
        executed = [{"error": str(exc)}]
    pending = [
        {"description": e["description"], "trigger": e["trigger"]}
        for e in _pending
    ]
    result = {"executed": executed, "pending": pending}
    print(json.dumps(result, indent=2))
    return result
