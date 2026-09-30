"""LLM-authored monitor support: validator, execution budget, context, monitor.

This module is deliberately free of any mcp_servers import. It will be reused
by an executor-side host later, so everything here depends only on the watchdog
package's own recorder/engine duck types and the standard library.

An authored monitor is a single agent-written function `check(ctx)` compiled
into a restricted namespace and evaluated on the engine's authored lane. The
restriction is layered: an AST validator rejects the constructs that could
reach outside the sandbox, a whitelisted __builtins__ removes the escape
hatches the validator cannot see (a name bound at runtime), and a line/time
budget bounds what a loop that passed both can still cost.
"""

from __future__ import annotations

import ast
import hashlib
import math
import sys
import time
from typing import Any, Callable

from .monitors import MonitorPrimitive
from .timebase import wall_ns

# Source ceiling: large enough for any reasonable predicate plus helpers,
# small enough that a runaway generation is refused before parsing.
MAX_SOURCE_CHARS = 8000
# Default per-call line budget for run_with_budget.
MAX_LINES_PER_CALL = 20000
# Window methods on MonitorContext return at most this many samples.
_MAX_WINDOW_SAMPLES = 1000
# Array-lane bounds. The rate cap keeps camera monitors from turning into a
# second render loop; the state cap keeps ctx.state from accumulating
# full-resolution frame history (a 640x360 float32 depth frame is ~0.9 MB, so
# 32 MB holds a handful of frames or plenty of ROIs, never a movie).
ARRAY_RATE_MAX_HZ = 2.0
ARRAY_STATE_MAX_BYTES = 32 * 1024 * 1024
# Valid values for a monitor's purpose. "clause" is a conformance monitor
# exactly as before; "observation" is a trigger for when to look, whose
# threshold is the agent's own choice rather than a specification bound.
MONITOR_PURPOSES = ("clause", "observation")

# Node types forbidden anywhere in authored source. Try is deliberately NOT
# here (a bare except is allowed); While is, because the line budget bounds
# a runaway for-loop far more predictably than a while whose condition never
# flips.
_FORBIDDEN_NODES: dict[type[ast.AST], str] = {
    ast.Import: "import",
    ast.ImportFrom: "import",
    ast.While: "while loop",
    ast.Global: "global statement",
    ast.Nonlocal: "nonlocal statement",
    ast.ClassDef: "class definition",
    ast.AsyncFunctionDef: "async function",
    ast.Await: "await",
    ast.Yield: "yield",
    ast.YieldFrom: "yield from",
}

# Names forbidden as any Name node. The whitelisted __builtins__ already
# excludes them; rejecting them at validation time turns a runtime NameError
# into an arm-time error with a line number.
_FORBIDDEN_NAMES = frozenset({
    "eval", "exec", "compile", "open", "getattr", "setattr", "delattr",
    "globals", "locals", "vars", "dir", "input", "breakpoint", "help",
    "memoryview", "type", "super", "object",
})

# The whole builtins surface authored code gets. `math` is injected beside it.
_ALLOWED_BUILTINS = {
    "abs": abs, "all": all, "any": any, "bool": bool, "dict": dict,
    "enumerate": enumerate, "float": float, "int": int, "len": len,
    "list": list, "max": max, "min": min, "range": range,
    "reversed": reversed, "round": round, "set": set, "sorted": sorted,
    "str": str, "sum": sum, "tuple": tuple, "zip": zip,
    "isinstance": isinstance,
    "ValueError": ValueError, "ZeroDivisionError": ZeroDivisionError,
}


class AuthoredCodeError(ValueError):
    """An authored monitor could not be built. Carries the stage that failed.

    stage is one of "validate", "compile", "first_call", "params", so the
    arm error tells the author WHERE to look, not only that something broke.
    lineno is the source line for validation failures, None elsewhere.
    """

    def __init__(self, stage: str, message: str, lineno: int | None = None) -> None:
        super().__init__(message)
        self.stage = stage
        self.lineno = lineno


class AuthoredBudgetExceeded(RuntimeError):
    """One check() call passed its line budget."""


class AuthoredReturnError(ValueError):
    """check() returned something outside its return contract."""


def _is_literal(node: ast.AST) -> bool:
    """True for a constant literal, including containers of literals."""
    if isinstance(node, ast.Constant):
        return True
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        return isinstance(node.operand, ast.Constant)
    if isinstance(node, (ast.Tuple, ast.List, ast.Set)):
        return all(_is_literal(e) for e in node.elts)
    if isinstance(node, ast.Dict):
        return all(k is not None and _is_literal(k) for k in node.keys) and all(
            _is_literal(v) for v in node.values
        )
    return False


def validate_source(source: str) -> None:
    """Reject authored source that steps outside the allowed subset.

    Raises AuthoredCodeError(stage="validate") naming the offending construct
    and its line. Rules: bounded size; top level holds only function
    definitions and literal-constant assignments; exactly one top-level
    `check` taking exactly one positional argument; no imports, while loops,
    classes, async, yield, global/nonlocal; no identifier starting with an
    underscore; none of the forbidden builtin names.
    """
    if not isinstance(source, str) or not source.strip():
        raise AuthoredCodeError("validate", "source must be a non-empty string.")
    if len(source) > MAX_SOURCE_CHARS:
        raise AuthoredCodeError(
            "validate",
            f"source is {len(source)} characters; the limit is {MAX_SOURCE_CHARS}.",
        )
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise AuthoredCodeError(
            "validate", f"syntax error: {exc.msg}", lineno=exc.lineno
        ) from exc

    check_defs: list[ast.FunctionDef] = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            if node.name == "check":
                check_defs.append(node)
            continue
        if isinstance(node, ast.Assign) and _is_literal(node.value):
            continue
        raise AuthoredCodeError(
            "validate",
            "top level may contain only function definitions and assignments "
            f"of literal constants; found {type(node).__name__}.",
            lineno=getattr(node, "lineno", None),
        )

    if len(check_defs) != 1:
        raise AuthoredCodeError(
            "validate",
            f"exactly one top-level function named 'check' is required; "
            f"found {len(check_defs)}.",
        )
    check = check_defs[0]
    args = check.args
    positional = len(args.posonlyargs) + len(args.args)
    if (
        positional != 1
        or args.vararg is not None
        or args.kwarg is not None
        or args.kwonlyargs
        or args.defaults
        or args.kw_defaults
    ):
        raise AuthoredCodeError(
            "validate",
            "check must take exactly one positional argument.",
            lineno=check.lineno,
        )

    for node in ast.walk(tree):
        for forbidden, label in _FORBIDDEN_NODES.items():
            if isinstance(node, forbidden):
                raise AuthoredCodeError(
                    "validate",
                    f"{label} is not allowed.",
                    lineno=getattr(node, "lineno", None),
                )
        if isinstance(node, ast.Name):
            if node.id.startswith("_"):
                raise AuthoredCodeError(
                    "validate",
                    f"names starting with '_' are not allowed: {node.id!r}.",
                    lineno=node.lineno,
                )
            if node.id in _FORBIDDEN_NAMES:
                raise AuthoredCodeError(
                    "validate",
                    f"{node.id!r} is not allowed.",
                    lineno=node.lineno,
                )
        elif isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            raise AuthoredCodeError(
                "validate",
                f"attributes starting with '_' are not allowed: {node.attr!r}.",
                lineno=node.lineno,
            )


def compile_check(
    source: str, extra_namespace: dict[str, Any] | None = None
) -> Callable[[Any], Any]:
    """Execute validated source in a restricted namespace; return its check().

    The namespace's __builtins__ is the whitelist above, plus the stdlib
    `math` module. extra_namespace adds provided (not imported) names -- the
    array lane injects numpy as `np` this way. Raises
    AuthoredCodeError(stage="compile") if execution fails or check is not
    callable afterwards.
    """
    namespace: dict[str, Any] = {
        "__builtins__": dict(_ALLOWED_BUILTINS),
        "math": math,
    }
    if extra_namespace:
        namespace.update(extra_namespace)
    try:
        exec(compile(source, "<authored_monitor>", "exec"), namespace)
    except Exception as exc:
        raise AuthoredCodeError("compile", f"{type(exc).__name__}: {exc}") from exc
    check = namespace.get("check")
    if not callable(check):
        raise AuthoredCodeError("compile", "source did not define a callable check.")
    return check


def run_with_budget(
    fn: Callable[[Any], Any], arg: Any, max_lines: int = MAX_LINES_PER_CALL
) -> tuple[Any, float]:
    """Call fn(arg) under a line-count budget on the current thread.

    Counts "line" trace events across every frame the call enters (the local
    tracer counts too), raising AuthoredBudgetExceeded when max_lines is
    passed. The previous tracer is always restored. Returns (result,
    elapsed_wall_seconds).

    A two-tuple, deliberately: this is the live lane's entry point and its shape
    is depended on by AuthoredMonitor and by tests. Callers that also need the
    line count -- the dry run, which reports it to the author -- use
    run_with_budget_counted below, which this delegates to.
    """
    result, elapsed, _lines = run_with_budget_counted(fn, arg, max_lines)
    return result, elapsed


def run_with_budget_counted(
    fn: Callable[[Any], Any],
    arg: Any,
    max_lines: int = MAX_LINES_PER_CALL,
    usage: dict[str, Any] | None = None,
) -> tuple[Any, float, int]:
    """run_with_budget, also returning how many lines were actually executed.

    Returns (result, elapsed_wall_seconds, lines_executed).

    `usage`, when given, is a dict this fills in before returning OR raising:
    {"lines": int, "wall_sec": float}. That is the only way to learn what a call
    cost when it ended in AuthoredBudgetExceeded or any other exception, and the
    dry run needs exactly that -- "your check blew the budget" is far less useful
    to an author than "it blew the budget after 20001 lines in 41 ms".

    Note what the line count does and does not measure: it counts Python "line"
    trace events. Work done inside a C extension emits none, so a single numpy
    call over a large array costs about one line here while taking arbitrary wall
    time. For anything array-shaped the wall clock is the real bound, not this.
    """
    count = 0

    def _local(frame: Any, event: str, _arg: Any):
        nonlocal count
        if event == "line":
            count += 1
            if count > max_lines:
                raise AuthoredBudgetExceeded(
                    f"line budget of {max_lines} exceeded in one call."
                )
        return _local

    def _global(frame: Any, event: str, _arg: Any):
        nonlocal count
        if event == "line":
            count += 1
            if count > max_lines:
                raise AuthoredBudgetExceeded(
                    f"line budget of {max_lines} exceeded in one call."
                )
        return _local

    previous = sys.gettrace()
    started = time.perf_counter()
    sys.settrace(_global)
    try:
        result = fn(arg)
    finally:
        # Restore the tracer FIRST, before any other work in this block, so a
        # failure while recording usage cannot leave the tracer installed.
        sys.settrace(previous)
        elapsed = time.perf_counter() - started
        if usage is not None:
            usage["lines"] = count
            usage["wall_sec"] = elapsed
    return result, elapsed, count


def json_safe(value: Any) -> Any:
    """Public alias for _json_safe.

    Exists because the dry run reports authored return values and ctx.state
    snapshots to the model, and both need the same coercion the arm path already
    applies to reference_values. A tool reaching for the underscored name would
    be the sort of private-API use this package's own validator forbids in
    authored code.
    """
    return _json_safe(value)


def _ndarray_summary(value: Any) -> dict[str, Any] | None:
    """A JSON-safe summary of an ndarray, or None if value is not one.

    Arrays must never be embedded in reference_values or any event payload --
    a single frame would dwarf every other record in the run -- so an array
    is reported as shape/dtype/range instead.
    """
    try:
        import numpy as np
    except Exception:
        return None
    if not isinstance(value, np.ndarray):
        return None
    out: dict[str, Any] = {
        "ndarray": [int(s) for s in value.shape],
        "dtype": str(value.dtype),
        "min": None, "max": None, "nan_frac": None,
    }
    try:
        if value.size:
            if value.dtype.kind == "f":
                finite = value[np.isfinite(value)]
                out["nan_frac"] = round(float(1.0 - finite.size / value.size), 6)
                if finite.size:
                    out["min"] = float(finite.min())
                    out["max"] = float(finite.max())
            elif value.dtype.kind in ("i", "u", "b"):
                out["nan_frac"] = 0.0
                out["min"] = float(value.min())
                out["max"] = float(value.max())
    except Exception:
        pass
    return out


def _json_safe(value: Any) -> Any:
    """A JSON-safe deep copy of an arm-time ctx.state value.

    int/float/str/bool/None are kept, except that a non-finite float becomes
    None (NaN/inf are not representable in strict JSON). Tuples and lists
    become lists, dicts recurse with keys coerced to str, an ndarray becomes
    the {"ndarray": [shape], "dtype", "min", "max", "nan_frac"} summary, and
    anything else is replaced by its repr().
    """
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    summary = _ndarray_summary(value)
    if summary is not None:
        return summary
    return repr(value)


def _state_ndarray_bytes(value: Any) -> int:
    """Total ndarray bytes held (recursively) in a ctx.state value."""
    nbytes = getattr(value, "nbytes", None)
    if nbytes is not None and hasattr(value, "dtype"):
        return int(nbytes)
    if isinstance(value, dict):
        return sum(_state_ndarray_bytes(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return sum(_state_ndarray_bytes(v) for v in value)
    return 0


def _xyz_copy(value: Any) -> dict[str, float]:
    return {axis: float(value[axis]) for axis in ("x", "y", "z")}


def _transform_to_dict(transform: Any) -> dict[str, Any]:
    """{"translation": {x,y,z}, "rotation": {x,y,z,w}} from either TF shape."""
    if isinstance(transform, dict):
        translation = transform.get("translation") or transform.get("position")
        rotation = transform.get("rotation") or transform.get("orientation")
        if translation is None and all(a in transform for a in ("x", "y", "z")):
            translation = transform
        out: dict[str, Any] = {"translation": _xyz_copy(translation)}
        out["rotation"] = (
            {axis: float(rotation[axis]) for axis in ("x", "y", "z", "w")}
            if isinstance(rotation, dict)
            else None
        )
        return out
    tf = getattr(transform, "transform", transform)
    translation = getattr(tf, "translation")
    rotation = getattr(tf, "rotation", None)
    return {
        "translation": {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
        },
        "rotation": (
            {
                "x": float(rotation.x),
                "y": float(rotation.y),
                "z": float(rotation.z),
                "w": float(rotation.w),
            }
            if rotation is not None
            else None
        ),
    }


def _copy_pose_sample(sample: dict[str, Any] | None) -> dict[str, Any] | None:
    if sample is None:
        return None
    out = dict(sample)
    pose = out.get("pose")
    if isinstance(pose, dict):
        out["pose"] = {
            key: dict(value) if isinstance(value, dict) else value
            for key, value in pose.items()
        }
    return out


class MonitorContext:
    """The only object authored code receives. Read-only views over recorder
    and engine state, plus the monitor's own params and state dict.

    Deliberately absent: current_step_id, the event stream, the registry, and
    the recorder/engine objects themselves. Every returned dict is a fresh
    copy. Every method call is noted, and channels_read exposes the sorted
    set as "method:arg" strings (bare "method" for calls with no argument).

    params carries the entries exactly as the monitor was armed with -- for an
    authored monitor each entry is the {"value": ..., "source": ...} mapping
    the tool received -- and value(name) is the convenience accessor for the
    plain scalar inside one.
    """

    def __init__(
        self,
        recorder: Any,
        engine: Any,
        params: dict[str, Any],
        state: dict[str, Any] | None = None,
        camera: Any | None = None,
        rate_hz: float | None = None,
    ) -> None:
        self._recorder = recorder
        self._engine = engine
        self.params = dict(params)
        self.state: dict[str, Any] = state if state is not None else {}
        self._channels: set[tuple[str, Any]] = set()
        # Camera access exists only on the array lane (rate_hz set). One
        # snapshot per tick: _begin_camera_tick clears it before each check
        # call, and the first camera accessor inside the call takes it, so
        # every accessor within one call returns the same frames AND the cost
        # of taking the snapshot lands inside the call's wall budget.
        self._camera = camera
        self._rate_hz = rate_hz
        self._cam_snapshot: dict[str, Any] | None = None

    # -- camera (array lane only) -------------------------------------------

    def _begin_camera_tick(self) -> None:
        self._cam_snapshot = None

    def _camera_snapshot(self) -> dict[str, Any]:
        if self._rate_hz is None:
            raise ValueError(
                "camera accessors (rgb/depth/camera/deproject/frame_stamps) "
                "are only available on the array lane: arm the monitor with "
                "rate_hz to use them."
            )
        if self._camera is None:
            raise ValueError(
                "no camera streams are attached to this monitor host, so the "
                "camera accessors cannot serve frames."
            )
        if self._cam_snapshot is None:
            self._cam_snapshot = self._camera.array_snapshot()
        return self._cam_snapshot

    def rgb(self) -> Any | None:
        """HxWx3 uint8 RGB (read-only), or None when absent/stale (>1 s)."""
        self._note("rgb")
        got = self._camera_snapshot().get("rgb")
        return got[0] if got is not None else None

    def depth(self) -> Any | None:
        """HxW float32 metres, NaN = no return (read-only), or None."""
        self._note("depth")
        got = self._camera_snapshot().get("depth")
        return got[0] if got is not None else None

    def camera(self) -> dict[str, Any] | None:
        """{width, height, K (3x3), frame_id, world_T_camera} or None."""
        self._note("camera")
        raw = self._camera_snapshot().get("camera")
        if raw is None:
            return None
        k = [float(v) for v in raw.get("K", [])]
        extr = raw.get("extrinsics_from_yaml") or {}
        return {
            "width": raw.get("width"),
            "height": raw.get("height"),
            "K": [k[0:3], k[3:6], k[6:9]] if len(k) >= 9 else None,
            "frame_id": raw.get("frame_id"),
            "world_T_camera": {
                "translation": [float(v) for v in (extr.get("xyz") or [])],
                "rotation_xyzw": [float(v) for v in (extr.get("quat_xyzw") or [])],
            },
        }

    def frame_stamps(self) -> dict[str, Any]:
        """{rgb_wall_ns, depth_wall_ns, age_ms} for this call's snapshot.

        age_ms is measured from the OLDEST present frame to the snapshot
        moment (the conservative staleness); None when neither frame exists.
        """
        self._note("frame_stamps")
        snap = self._camera_snapshot()
        rgb, depth = snap.get("rgb"), snap.get("depth")
        stamps = [got[1] for got in (rgb, depth) if got is not None]
        age_ms = (
            round((snap["taken_wall_ns"] - min(stamps)) / 1e6, 3) if stamps else None
        )
        return {
            "rgb_wall_ns": rgb[1] if rgb is not None else None,
            "depth_wall_ns": depth[1] if depth is not None else None,
            "age_ms": age_ms,
        }

    def deproject(self, u: Any, v: Any, z: Any) -> Any:
        """Pixel(s) + depth -> world point(s), through K and world_T_camera.

        Scalars return {"x", "y", "z"}; array inputs return an Nx3 ndarray.
        Raises when camera_info (and so K / extrinsics) is unavailable.
        """
        import numpy as np

        self._note("deproject")
        cam = self.camera()
        if cam is None or cam.get("K") is None:
            raise ValueError("deproject needs camera_info, which is unavailable.")
        k = cam["K"]
        fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
        pose = cam["world_T_camera"]
        t = np.asarray(pose["translation"], dtype=float)
        qx, qy, qz, qw = (float(v_) for v_ in pose["rotation_xyzw"])
        rot = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        u_arr = np.asarray(u, dtype=float)
        v_arr = np.asarray(v, dtype=float)
        z_arr = np.asarray(z, dtype=float)
        x_cam = (u_arr - cx) * z_arr / fx
        y_cam = (v_arr - cy) * z_arr / fy
        cam_pts = np.stack(
            [np.ravel(x_cam), np.ravel(y_cam), np.ravel(np.broadcast_to(z_arr, x_cam.shape))],
            axis=1,
        )
        world = cam_pts @ rot.T + t
        if u_arr.ndim == 0 and v_arr.ndim == 0:
            return {"x": float(world[0, 0]), "y": float(world[0, 1]), "z": float(world[0, 2])}
        return world

    # -- bookkeeping -------------------------------------------------------

    def _note(self, method: str, first_arg: Any = None) -> None:
        self._channels.add((method, first_arg))

    @property
    def channels_read(self) -> list[str]:
        return sorted(
            f"{method}:{arg}" if arg is not None else method
            for method, arg in self._channels
        )

    def value(self, name: str) -> Any:
        """The plain value of params[name]: its "value" entry if the param is
        a {value, source} mapping, else the entry itself."""
        entry = self.params[name]
        if isinstance(entry, dict) and "value" in entry:
            return entry["value"]
        return entry

    # -- identity listings -------------------------------------------------

    def object_ids(self) -> list[str]:
        self._note("object_ids")
        return list(self._recorder.object_ids())

    def frame_ids(self) -> list[str]:
        self._note("frame_ids")
        return list(self._recorder.frame_ids())

    def joint_names(self) -> list[str]:
        self._note("joint_names")
        latest = self._recorder.latest_joint_state()
        if latest is None:
            return []
        return sorted(set(latest.get("positions", {})) | set(latest.get("efforts", {})))

    # -- latest samples ----------------------------------------------------

    def object_pose(self, object_id: str) -> dict[str, Any] | None:
        self._note("object_pose", object_id)
        return _copy_pose_sample(self._recorder.latest_object_pose(object_id))

    def frame_pose(self, frame_id: str) -> dict[str, Any] | None:
        self._note("frame_pose", frame_id)
        return _copy_pose_sample(self._recorder.latest_frame_pose(frame_id))

    def transform(self, parent: str, child: str) -> dict[str, Any]:
        self._note("transform", f"{parent}->{child}")
        return _transform_to_dict(self._recorder.lookup_transform(parent, child))

    def joint(self, name: str) -> dict[str, Any] | None:
        self._note("joint", name)
        latest = self._recorder.latest_joint_state()
        if latest is None:
            return None
        positions = latest.get("positions", {})
        efforts = latest.get("efforts", {})
        if name not in positions and name not in efforts:
            return None
        return {
            "wall_ns": latest.get("wall_ns"),
            "position": positions.get(name),
            "effort": efforts.get(name),
        }

    # -- windows -----------------------------------------------------------

    def joint_window(self, name: str, seconds: float) -> list[dict[str, Any]]:
        self._note("joint_window", name)
        start, end = self._window_bounds(seconds)
        rows = self._recorder.get_joint_window(start, end, [name], _MAX_WINDOW_SAMPLES)
        return [
            {
                "wall_ns": row.get("wall_ns"),
                "position": row.get("positions", {}).get(name),
                "effort": row.get("efforts", {}).get(name),
            }
            for row in rows
        ]

    def object_pose_window(self, object_id: str, seconds: float) -> list[dict[str, Any]]:
        self._note("object_pose_window", object_id)
        start, end = self._window_bounds(seconds)
        rows = self._recorder.get_object_pose_window(
            start, end, object_id, _MAX_WINDOW_SAMPLES
        )
        return [_copy_pose_sample(row) for row in rows]

    def frame_pose_window(self, frame_id: str, seconds: float) -> list[dict[str, Any]]:
        self._note("frame_pose_window", frame_id)
        start, end = self._window_bounds(seconds)
        rows = self._recorder.get_frame_window(start, end, frame_id, _MAX_WINDOW_SAMPLES)
        return [_copy_pose_sample(row) for row in rows]

    # -- cell state and helpers --------------------------------------------

    def held_object(self) -> str | None:
        self._note("held_object")
        return self._engine.held_object_id()

    def now_wall_ns(self) -> int:
        self._note("now_wall_ns")
        return wall_ns()

    def dist(self, a: dict[str, Any], b: dict[str, Any]) -> float:
        return math.sqrt(
            sum((float(a[axis]) - float(b[axis])) ** 2 for axis in ("x", "y", "z"))
        )

    @staticmethod
    def _window_bounds(seconds: float) -> tuple[int, int]:
        span = float(seconds)
        if not math.isfinite(span) or span <= 0.0:
            raise ValueError("seconds must be a finite number > 0.")
        end = wall_ns()
        return end - int(span * 1_000_000_000), end


class AuthoredMonitor(MonitorPrimitive):
    """One agent-authored predicate, evaluated on the engine's authored lane.

    Return contract for check(ctx): None means the condition holds; a dict
    with a finite numeric "value" (and optional str "detail") is a violation
    and fires with that value; anything else is AuthoredReturnError.

    The first check(ctx) call happens at construction, before the monitor is
    armed. It is the arm-time reference capture: authored code that needs a
    frozen reference stores it in ctx.state on that call, and its return
    value (kept as first_call_result) never fires. ctx.state as it stands
    after the arm-time call is returned as reference_values.
    """

    feed = "authored"
    origin = "authored"

    def __init__(
        self,
        source: str,
        description: str,
        params: dict[str, Any],
        recorder: Any,
        engine: Any,
        object_id: str | None = None,
        threshold_param: str | None = None,
        camera: Any | None = None,
        rate_hz: float | None = None,
        purpose: str = "clause",
        threshold_rationale: str | None = None,
        **kwargs: Any,
    ) -> None:
        if not isinstance(params, dict):
            raise AuthoredCodeError("params", "params must be a dict.")
        super().__init__("authored", params, **kwargs)
        if not isinstance(description, str) or not description.strip():
            raise AuthoredCodeError("params", "description must be a non-empty string.")
        if purpose not in MONITOR_PURPOSES:
            raise AuthoredCodeError(
                "params", f"purpose must be one of {list(MONITOR_PURPOSES)}."
            )
        self.source = source
        self.description = description.strip()
        self.object_id = object_id
        self.strikes = 0
        self.last_detail: str | None = None
        self.last_wall_time_sec = 0.0
        self.purpose = purpose
        self.threshold_rationale = threshold_rationale
        # rate_hz selects the ARRAY lane: a distinct feed so neither
        # _evaluate_feed (tf/joint_states) nor the scalar authored lane
        # (feed == "authored") ever evaluates this monitor.
        self.rate_hz = float(rate_hz) if rate_hz is not None else None
        if self.rate_hz is not None:
            if not math.isfinite(self.rate_hz) or self.rate_hz <= 0:
                raise AuthoredCodeError("params", "rate_hz must be a finite number > 0.")
            self.feed = "authored_array"
        # Array-lane scheduling/stats, owned by the engine's array worker.
        self.next_due_ns = 0
        self.last_wall_ms: float | None = None
        self.calls = 0

        plain, sources = self._parse_params(params)
        self._param_sources = sources

        validate_source(source)
        self.code_sha256 = hashlib.sha256(source.encode("utf-8")).hexdigest()
        extra_namespace: dict[str, Any] | None = None
        if self.rate_hz is not None:
            # numpy is PROVIDED on the array lane, never imported by authored
            # code (import stays forbidden by the validator).
            try:
                import numpy as np
            except Exception as exc:  # pragma: no cover - numpy is a hard dep
                raise AuthoredCodeError(
                    "compile", f"numpy is unavailable for the array lane: {exc}"
                ) from exc
            extra_namespace = {"np": np}
        self._check = compile_check(source, extra_namespace)

        # Threshold plumbing, shaped so the UNCHANGED
        # _resolve_threshold_provenance in the server can run against this
        # monitor exactly as it runs against a library one.
        self.threshold_param = threshold_param
        self.threshold_registry_field = None
        if threshold_param is not None:
            if threshold_param not in plain:
                raise AuthoredCodeError(
                    "params",
                    f"threshold_param {threshold_param!r} does not name a key in "
                    f"params ({sorted(plain)}).",
                )
            try:
                value = float(plain[threshold_param])
            except (TypeError, ValueError):
                value = float("nan")
            if not math.isfinite(value):
                raise AuthoredCodeError(
                    "params",
                    f"params[{threshold_param!r}] must carry a finite numeric value.",
                )
            self.threshold = value
            source_path = sources.get(threshold_param)
            if isinstance(source_path, str) and source_path.startswith("raw_spec."):
                self.threshold_registry_field = source_path[len("raw_spec."):]
        else:
            self.threshold = 0.0
        self.comparison = "authored"

        # ctx.params carries the {value, source} entries EXACTLY as armed (a
        # JSON-safe deep copy), matching the tool docstring authored code is
        # written against; plain values above serve only the threshold
        # plumbing. One ctx for the monitor's whole life: the arm-time first
        # call and every later evaluation share this object, so ctx.state
        # persists across calls.
        self.ctx = MonitorContext(
            recorder, engine, _json_safe(dict(params)), state={},
            camera=camera, rate_hz=self.rate_hz,
        )
        try:
            self.ctx._begin_camera_tick()
            self.first_call_result, self.last_wall_time_sec = run_with_budget(
                self._check, self.ctx
            )
        except AuthoredCodeError:
            raise
        except Exception as exc:
            raise AuthoredCodeError(
                "first_call", f"{type(exc).__name__}: {exc}"
            ) from exc
        # The arm-time reference capture, in the same field the library
        # primitives use: engine.arm already returns and publishes
        # reference_values, so no new plumbing is needed for it.
        self.reference_values = _json_safe(dict(self.ctx.state))

    @staticmethod
    def _parse_params(
        params: dict[str, Any],
    ) -> tuple[dict[str, Any], dict[str, str | None]]:
        """Split {name: {"value": v, "source": s|None}} into values and sources."""
        plain: dict[str, Any] = {}
        sources: dict[str, str | None] = {}
        for name, entry in params.items():
            if not isinstance(entry, dict) or "value" not in entry:
                raise AuthoredCodeError(
                    "params",
                    f"params[{name!r}] must be an object with a 'value' key "
                    f"(and an optional 'source').",
                )
            source = entry.get("source")
            if source is not None and not isinstance(source, str):
                raise AuthoredCodeError(
                    "params", f"params[{name!r}].source must be a string or null."
                )
            plain[name] = entry["value"]
            sources[name] = source
        return plain, sources

    @property
    def subject_object_id(self) -> str | None:
        """The declared subject; None lets the engine resolve the held part."""
        return self.object_id

    @property
    def channels_read(self) -> list[str]:
        return self.ctx.channels_read

    @property
    def param_sources(self) -> dict[str, str | None]:
        return dict(self._param_sources)

    def dry_call(
        self,
        max_lines: int = MAX_LINES_PER_CALL,
        usage: dict[str, Any] | None = None,
    ) -> Any:
        """Run check(ctx) once under budget and return its RAW return value.

        For dry runs only. Three deliberate differences from evaluate():

        - The return contract is NOT applied. evaluate() turns a return into a
          fire value or raises AuthoredReturnError; here the author gets back
          exactly what their function returned, because seeing that it returned
          the wrong SHAPE is half the point of a dry run.
        - No monitor state is touched. strikes, last_detail and
          last_wall_time_sec are left alone, so a dry run cannot push a monitor
          towards the strike limit or leave stale fields behind.
        - Nothing fires and nothing is recorded. The caller owns the reporting.

        ctx is the monitor's own context, so ctx.state persists across
        successive dry_call()s exactly as it would across live evaluations --
        which is what lets an author watch state accumulate tick by tick.
        """
        self.ctx._begin_camera_tick()
        result, _elapsed, _lines = run_with_budget_counted(
            self._check, self.ctx, max_lines, usage
        )
        return result

    def evaluate(self, sample: Any) -> float | None:
        """Run check(ctx) under budget and apply the return contract."""
        self.ctx._begin_camera_tick()
        result, self.last_wall_time_sec = run_with_budget(self._check, self.ctx)
        self.calls += 1
        self.last_wall_ms = round(self.last_wall_time_sec * 1000.0, 3)
        if self.rate_hz is not None:
            held = _state_ndarray_bytes(self.ctx.state)
            if held > ARRAY_STATE_MAX_BYTES:
                raise AuthoredReturnError(
                    f"ctx.state holds {held} bytes of arrays, over the "
                    f"{ARRAY_STATE_MAX_BYTES} byte cap -- store a downsampled "
                    f"ROI or a summary, not full-resolution frame history."
                )
        if result is None:
            return None
        if isinstance(result, dict) and "value" in result:
            try:
                value = float(result["value"])
            except (TypeError, ValueError):
                value = float("nan")
            if not math.isfinite(value):
                raise AuthoredReturnError(
                    f"check returned a non-finite 'value': {result.get('value')!r}."
                )
            detail = result.get("detail")
            if detail is not None and not isinstance(detail, str):
                raise AuthoredReturnError("check's 'detail' must be a string.")
            self.last_detail = detail
            return value
        raise AuthoredReturnError(
            "check must return None, or a dict with a finite numeric 'value' "
            f"and an optional str 'detail'; got {type(result).__name__}."
        )


__all__ = [
    "ARRAY_RATE_MAX_HZ",
    "ARRAY_STATE_MAX_BYTES",
    "AuthoredCodeError",
    "AuthoredBudgetExceeded",
    "AuthoredReturnError",
    "AuthoredMonitor",
    "MONITOR_PURPOSES",
    "MonitorContext",
    "MAX_LINES_PER_CALL",
    "MAX_SOURCE_CHARS",
    "compile_check",
    "json_safe",
    "run_with_budget",
    "run_with_budget_counted",
    "validate_source",
]
