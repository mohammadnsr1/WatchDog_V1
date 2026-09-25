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


def compile_check(source: str) -> Callable[[Any], Any]:
    """Execute validated source in a restricted namespace; return its check().

    The namespace's __builtins__ is the whitelist above, plus the stdlib
    `math` module. Raises AuthoredCodeError(stage="compile") if execution
    fails or check is not callable afterwards.
    """
    namespace: dict[str, Any] = {
        "__builtins__": dict(_ALLOWED_BUILTINS),
        "math": math,
    }
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
        sys.settrace(previous)
    return result, time.perf_counter() - started


def _json_safe(value: Any) -> Any:
    """A JSON-safe deep copy of an arm-time ctx.state value.

    int/float/str/bool/None are kept, except that a non-finite float becomes
    None (NaN/inf are not representable in strict JSON). Tuples and lists
    become lists, dicts recurse with keys coerced to str, and anything else
    is replaced by its repr().
    """
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    return repr(value)


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
    """

    def __init__(
        self,
        recorder: Any,
        engine: Any,
        params: dict[str, Any],
        state: dict[str, Any] | None = None,
    ) -> None:
        self._recorder = recorder
        self._engine = engine
        self.params = dict(params)
        self.state: dict[str, Any] = state if state is not None else {}
        self._channels: set[tuple[str, Any]] = set()

    # -- bookkeeping -------------------------------------------------------

    def _note(self, method: str, first_arg: Any = None) -> None:
        self._channels.add((method, first_arg))

    @property
    def channels_read(self) -> list[str]:
        return sorted(
            f"{method}:{arg}" if arg is not None else method
            for method, arg in self._channels
        )

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
        **kwargs: Any,
    ) -> None:
        if not isinstance(params, dict):
            raise AuthoredCodeError("params", "params must be a dict.")
        super().__init__("authored", params, **kwargs)
        if not isinstance(description, str) or not description.strip():
            raise AuthoredCodeError("params", "description must be a non-empty string.")
        self.source = source
        self.description = description.strip()
        self.object_id = object_id
        self.strikes = 0
        self.last_detail: str | None = None
        self.last_wall_time_sec = 0.0

        plain, sources = self._parse_params(params)
        self._param_sources = sources

        validate_source(source)
        self.code_sha256 = hashlib.sha256(source.encode("utf-8")).hexdigest()
        self._check = compile_check(source)

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

        self.ctx = MonitorContext(recorder, engine, plain, state={})
        try:
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

    def evaluate(self, sample: Any) -> float | None:
        """Run check(ctx) under budget and apply the return contract."""
        result, self.last_wall_time_sec = run_with_budget(self._check, self.ctx)
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
    "AuthoredCodeError",
    "AuthoredBudgetExceeded",
    "AuthoredReturnError",
    "AuthoredMonitor",
    "MonitorContext",
    "MAX_LINES_PER_CALL",
    "MAX_SOURCE_CHARS",
    "compile_check",
    "run_with_budget",
    "validate_source",
]
