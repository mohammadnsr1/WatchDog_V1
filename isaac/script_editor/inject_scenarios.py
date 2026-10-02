"""Scene-observation scenarios for the live Isaac cell (Script Editor tool).

Builds ON inject_intruder.py (imported, not copied): same session-dir
resolution (~/.watchdog/current_session), same ground-truth writer, same
camera model, same /World/WatchdogIntruders root (outside the TF targetPrims
list -- keep it that way).

USAGE (paste into Isaac Sim's Script Editor; reload-safe):

    import sys, importlib
    sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1/isaac/script_editor")
    import inject_intruder, inject_scenarios
    importlib.reload(inject_intruder); importlib.reload(inject_scenarios)
    from inject_scenarios import *

    presets()      # prints one ready-to-paste call per scenario
    cancel_all()   # stops every callback and removes every prim this module made

GROUND-TRUTH RECORD SCHEMA (one JSON line per record in
<session_dir>/ground_truth_injections.jsonl). Every record carries ALL of
inject_intruder.build_ground_truth_record's keys:

    action, name, prim_path, shape, size_m, position_xyz_world, yaw_deg,
    color_rgb, physics, label, expected_impact, notes, trigger, wall_ns,
    wall_time_iso, session_dir, script_sha256,
    pixel_uv, in_camera_view, edge_margin_px   <- computed AT the action
                                                  (for removals: the last
                                                  position before removal)

plus, on every record this module writes:

    scenario      -- "place_object" | "visit" | "nudge_object" | "remove"
                     | "cancel_all"
    event_kind    -- "foreign_object" | "transient_visitor"
                     | "external_displacement" | "removal"
    episode_id    -- one id shared by every record of one scenario run
    scenarios_sha256 -- sha256 of THIS file (script_sha256 stays
                     inject_intruder's, since its writer stamps it)

per-event_kind actions (the `action` field):

    foreign_object        : "place"
    transient_visitor     : "enter_start", "in_view", "dwell_start",
                            "leave_start", "out_of_view", "removed"
                            (in_view/out_of_view fire on the FIRST frame the
                            projected centre crosses the view edge; they are
                            skipped if the path never crosses it)
    external_displacement : "nudge", with extras pre_pose_world /
                            post_pose_world ({position:[x,y,z],
                            orientation_xyzw:[x,y,z,w]})
    removal               : "removed" (remove()/cancel_all()); cancel_all
                            also writes "cancelled" for tasks stopped before
                            they ever created a prim

visit() extras: dwell_position_world, dwell_s, speed_mps, collide,
entry_offset_world.

expected_impact here is "benign" | "violation" (NOTE: inject_intruder's own
inject() keeps its historical "violates" vocabulary; records store whichever
the caller's API took).

Everything time-driven (after_s, at_step, the visit motion, the robot-idle
guard) runs on ONE omni.kit.app update subscription held in a module-level
runtime; importlib.reload cancels the previous subscription before installing
a new one, and cancel_all() tears everything down. No time.sleep on the main
thread, ever.

Pure helpers (projection checks, start parsing, geometry, record building,
presets) import no omni/pxr and are unit-tested offline in
test_inject_scenarios.py; the Isaac layer imports lazily inside functions.
"""

from __future__ import annotations

import hashlib
import json
import time
import uuid
from pathlib import Path
from typing import Any, Callable

import inject_intruder as ii

# ---------------------------------------------------------------------------
# Pure core
# ---------------------------------------------------------------------------

EVENT_KINDS = (
    "foreign_object", "transient_visitor", "external_displacement", "removal",
)
SCENARIO_IMPACTS = ("benign", "violation")
VISIT_SHAPES = ("hand", "person")
VISIT_PHASES = (
    "enter_start", "in_view", "dwell_start", "leave_start", "out_of_view",
    "removed",
)

MIN_EDGE_MARGIN_PX = 60.0
IDLE_SPEED_RAD_S = 0.01
IDLE_TIMEOUT_S = 30.0

# "hand": a forearm-sized box sliding in horizontally; long axis along the
# motion. "person": a capsule standing at the bench edge.
HAND_BOX_XYZ = (0.40, 0.08, 0.08)
HAND_HEIGHT_ABOVE_BENCH = 0.15
PERSON_RADIUS = 0.18
PERSON_HEIGHT = 1.0  # the capsule's cylindrical section; total = height + 2r


def this_script_sha256() -> str:
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def new_episode_id() -> str:
    return f"ep_{uuid.uuid4().hex[:6]}"


class ViewRefusal(ValueError):
    """Raised when a planned position fails the camera-view check."""


def check_view(
    point_xyz_world: Any,
    what: str,
    min_edge_margin_px: float = MIN_EDGE_MARGIN_PX,
    allow_out_of_view: bool = False,
) -> dict[str, Any]:
    """camera_view_fields for a PLANNED position, refusing when out of view.

    Refuses (ViewRefusal, reason printed) when the centre is out of view or
    its edge margin is under min_edge_margin_px, unless allow_out_of_view.
    The 2026-10-01 smoke intruders at x ~ -0.5 sat on the image edge; a miss
    on an invisible object must never score against the agent, so planning
    out-of-view placements is opt-in, not a default accident.
    """
    fields = ii.camera_view_fields(point_xyz_world)
    margin = fields["edge_margin_px"]
    bad = (not fields["in_camera_view"]) or margin is None or margin < float(
        min_edge_margin_px
    )
    if bad and not allow_out_of_view:
        reason = (
            f"{what} at {tuple(point_xyz_world)} projects to "
            f"{fields['pixel_uv']} (edge margin {margin} px, need >= "
            f"{min_edge_margin_px}). Pass allow_out_of_view=True to place it "
            f"anyway."
        )
        print(f"[inject_scenarios] REFUSED: {reason}")
        raise ViewRefusal(reason)
    if bad:
        print(
            f"[inject_scenarios] WARNING: {what} is out of the camera's "
            f"reliable view ({fields['pixel_uv']}, margin {margin} px); "
            f"proceeding because allow_out_of_view=True."
        )
    return fields


def parse_start(start: str) -> tuple[str, Any]:
    """("now", None) | ("after_s", float) | ("at_step", step_id) from the
    common `start` parameter ("now" | "after_s=<float>" | "at_step=<id>")."""
    if start == "now":
        return ("now", None)
    if isinstance(start, str) and start.startswith("after_s="):
        try:
            delay = float(start[len("after_s="):])
        except ValueError:
            raise ValueError(f"bad start {start!r}: after_s needs a number.")
        if delay < 0:
            raise ValueError(f"bad start {start!r}: after_s must be >= 0.")
        return ("after_s", delay)
    if isinstance(start, str) and start.startswith("at_step="):
        step_id = start[len("at_step="):]
        if not step_id:
            raise ValueError(f"bad start {start!r}: at_step needs a step_id.")
        return ("at_step", step_id)
    raise ValueError(
        f"start must be 'now', 'after_s=<float>' or 'at_step=<step_id>', "
        f"got {start!r}."
    )


def start_trigger_text(start: str) -> str:
    mode, value = parse_start(start)
    if mode == "now":
        return "immediate"
    if mode == "after_s":
        return f"delay:{value:g}"
    return f"at_step:{value}"


def validate_scenario_impact(expected_impact: str) -> str:
    if expected_impact not in SCENARIO_IMPACTS:
        raise ValueError(
            f"expected_impact must be one of {list(SCENARIO_IMPACTS)}, "
            f"got {expected_impact!r}."
        )
    return expected_impact


def visit_geometry(
    shape: str,
    dwell_position: Any,
    entry_offset: Any | None = None,
) -> dict[str, Any]:
    """Pure geometry for a visit: sizes, start and dwell centres.

    dwell_position is (x, y) or (x, y, z); z defaults per shape -- the hand
    slides at a fixed height above the bench, the person's capsule centre is
    half its total height above the bench. entry_offset (world xyz, added to
    the dwell centre to give the ENTER start point) defaults to a straight
    slide-in from the x- edge far enough to start out of view.
    """
    if shape not in VISIT_SHAPES:
        raise ValueError(f"shape must be one of {list(VISIT_SHAPES)}, got {shape!r}.")
    values = list(dwell_position)
    if len(values) == 2:
        values.append(None)
    x, y, z = values
    if shape == "hand":
        z = ii.BENCH_TOP_Z + HAND_HEIGHT_ABOVE_BENCH if z is None else float(z)
        size = {"box_xyz": HAND_BOX_XYZ}
        default_offset = (-1.2, 0.0, 0.0)
    else:
        total = PERSON_HEIGHT + 2 * PERSON_RADIUS
        z = ii.BENCH_TOP_Z + total / 2.0 if z is None else float(z)
        size = {"radius": PERSON_RADIUS, "height": PERSON_HEIGHT}
        default_offset = (0.0, -0.6, 0.0)
    dwell = (float(x), float(y), float(z))
    off = ii._xyz(entry_offset, "entry_offset") if entry_offset is not None \
        else default_offset
    start = (dwell[0] + off[0], dwell[1] + off[1], dwell[2] + off[2])
    return {"shape": shape, "dwell": dwell, "start": start, "size": size,
            "entry_offset": off}


def scenario_record(
    *,
    scenario: str,
    event_kind: str,
    episode_id: str,
    action: str,
    name: str,
    prim_path: str,
    session_dir: str,
    trigger: str = "immediate",
    position_xyz_world: Any | None = None,
    shape: str | None = None,
    size_m: float | None = None,
    yaw_deg: float | None = None,
    color_rgb: Any | None = None,
    physics: bool | None = None,
    label: str | None = None,
    expected_impact: str | None = None,
    notes: str = "",
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """One scenario ground-truth record: inject_intruder's full schema plus
    scenario / event_kind / episode_id / scenarios_sha256 (+ extras).

    The visibility triple (pixel_uv, in_camera_view, edge_margin_px) comes
    from the builder, computed from position_xyz_world AT this moment --
    callers pass the CURRENT position (for removals: the last one before
    removal)."""
    if event_kind not in EVENT_KINDS:
        raise ValueError(
            f"event_kind must be one of {list(EVENT_KINDS)}, got {event_kind!r}."
        )
    record = ii.build_ground_truth_record(
        action=action, name=name, prim_path=prim_path, trigger=trigger,
        wall_ns=time.time_ns(), session_dir=session_dir,
        shape=shape, size_m=size_m,
        position_xyz_world=tuple(position_xyz_world) if position_xyz_world
        is not None else None,
        yaw_deg=yaw_deg,
        color_rgb=tuple(color_rgb) if color_rgb is not None else None,
        physics=physics, label=label, expected_impact=expected_impact,
        notes=notes,
    )
    record["scenario"] = scenario
    record["event_kind"] = event_kind
    record["episode_id"] = episode_id
    record["scenarios_sha256"] = this_script_sha256()
    if extra:
        record.update(extra)
    return record


# ---------------------------------------------------------------------------
# Preset positions, verified OFFLINE against the camera model and the
# base1-clearance rule (registry world poses are not available offline, so
# clearance uses the fallback rule: >= 0.12 m from base1 at (0.79, -0.20)).
# test_inject_scenarios.py re-verifies every one of these.
# ---------------------------------------------------------------------------

BASE1_XY = (0.79, -0.20)
MIN_BASE1_CLEARANCE_M = 0.12

PRESETS = {
    "place_object": {
        "position": (0.45, -0.05, None),   # margin ~71 px, 0.37 m from base1
        "call": (
            'place_object(position_xyz_world=(0.45, -0.05, None),\n'
            '             label="foreign cube on free bench area",\n'
            '             expected_impact="benign")'
        ),
    },
    "visit_hand": {
        "position": (0.45, -0.10, ii.BENCH_TOP_Z + HAND_HEIGHT_ABOVE_BENCH),
        "call": (
            'visit(shape="hand", dwell_position=(0.45, -0.10), dwell_s=4.0,\n'
            '      speed_mps=0.3, label="forearm over the work area",\n'
            '      expected_impact="benign")'
        ),
    },
    "visit_person": {
        "position": (0.10, -0.45, 0.51),   # capsule, partly in view
        "call": (
            'visit(shape="person", dwell_position=(0.10, -0.45, 0.51),\n'
            '      dwell_s=6.0, speed_mps=0.5,\n'
            '      label="person at the bench edge",\n'
            '      expected_impact="benign")'
        ),
    },
    "nudge_object": {
        "position": None,  # targets an existing object; no placement check
        "call": (
            'nudge_object("u_orange", delta_xyz=(0.03, 0.0, 0.0),\n'
            '             label="bystander displaced by an external cause",\n'
            '             expected_impact="violation")'
        ),
    },
}


def presets() -> None:
    """Print one ready-to-paste Script Editor call per scenario.

    Positions were checked offline to be in camera view with >= 60 px edge
    margin and >= 0.12 m from base1 at (0.79, -0.20); the reload preamble is
    included so each snippet works pasted alone.
    """
    preamble = (
        "import sys, importlib\n"
        'sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1/isaac/script_editor")\n'
        "import inject_intruder, inject_scenarios\n"
        "importlib.reload(inject_intruder); importlib.reload(inject_scenarios)\n"
        "from inject_scenarios import *\n"
    )
    print("# --- paste the preamble once, then any scenario call ---")
    print(preamble)
    for key, preset in PRESETS.items():
        print(f"# {key}")
        print(preset["call"])
        print()
    print("# stop everything and remove every prim this module created:")
    print("cancel_all()")


# ---------------------------------------------------------------------------
# Isaac layer. All omni/pxr/isaacsim imports are lazy. The runtime (the one
# update subscription, the task list, the created-prim registry) survives in
# this module's dict across importlib.reload: reload re-executes the module
# in the SAME dict, so the previous runtime is still reachable here and is
# cancelled before a fresh one takes its place.
# ---------------------------------------------------------------------------

class _Runtime:
    def __init__(self) -> None:
        self.subscription: Any = None
        self.tasks: list[dict[str, Any]] = []
        # name -> {scenario, event_kind, episode_id, prim_path}
        self.created: dict[str, dict[str, str]] = {}
        self.articulation: Any = None
        self.articulation_path: str | None = None

    def cancel_subscription(self) -> None:
        self.subscription = None  # dropping the handle unsubscribes


_previous = globals().get("_RUNTIME")
if _previous is not None:
    try:
        _previous.cancel_subscription()
        print("[inject_scenarios] reload: cancelled the previous runtime "
              f"({len(_previous.tasks)} pending task(s) dropped; prims left "
              "in place -- cancel_all() removes them)")
    except Exception:
        pass
_RUNTIME = _Runtime()
if _previous is not None and getattr(_previous, "created", None):
    # Prims created before the reload stay OURS to clean up.
    _RUNTIME.created.update(_previous.created)


def _ensure_subscription() -> None:
    if _RUNTIME.subscription is not None:
        return
    import omni.kit.app

    stream = omni.kit.app.get_app().get_update_event_stream()
    _RUNTIME.subscription = stream.create_subscription_to_pop(
        _on_update, name="watchdog_scenarios"
    )


# --- robot-idle guard -------------------------------------------------------

def _find_robot_articulation_path() -> str:
    """The robot articulation root's prim path, discovered from the stage.

    Not hardcoded: searches for prims carrying UsdPhysics.ArticulationRootAPI
    (the UR5e is the cell's only articulation; if several exist, the one whose
    path mentions 'ur' wins and the choice is printed)."""
    from pxr import UsdPhysics

    stage = ii._stage()
    roots = [
        str(prim.GetPath()) for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if not roots:
        raise RuntimeError(
            "no articulation root found on the stage; cannot run the "
            "robot-idle guard. Pass idle_guard=False to skip it."
        )
    chosen = next((r for r in roots if "ur" in r.lower()), roots[0])
    if len(roots) > 1:
        print(f"[inject_scenarios] articulations {roots}; idle guard uses {chosen}")
    return chosen


def _joint_speeds_rad_s() -> list[float] | None:
    """Current |joint velocity| per joint, or None when they cannot be read.

    Uses the same API the repo's script_editor pattern uses
    (isaacsim.core.prims.Articulation; see script_editor_correct_pattern.py)."""
    try:
        if _RUNTIME.articulation is None:
            from isaacsim.core.prims import Articulation

            path = _find_robot_articulation_path()
            _RUNTIME.articulation = Articulation(
                prim_paths_expr=path, name="watchdog_scenarios_robot"
            )
            _RUNTIME.articulation_path = path
        velocities = _RUNTIME.articulation.get_joint_velocities()
        if velocities is None:
            return None
        flat = list(getattr(velocities, "flatten", lambda: velocities)())
        return [abs(float(v)) for v in flat]
    except Exception:
        return None


def _robot_is_idle() -> bool | None:
    """True/False from live joint speeds; None when unreadable."""
    speeds = _joint_speeds_rad_s()
    if speeds is None:
        return None
    return all(s < IDLE_SPEED_RAD_S for s in speeds)


# --- the per-frame task machine ---------------------------------------------

def _make_step_tail(step_id: str) -> dict[str, Any]:
    return {"matcher": ii.StepTriggerMatcher(step_id), "path": None, "offset": 0}


def _step_tail_ready(tail: dict[str, Any]) -> bool:
    if tail["path"] is None:
        try:
            tail["path"] = ii.resolve_session_dir() / "events_scene.jsonl"
        except RuntimeError:
            return False
    path = tail["path"]
    if not path.exists():
        return False
    with open(path, "r", encoding="utf-8") as handle:
        handle.seek(tail["offset"])
        chunk = handle.read()
        tail["offset"] = handle.tell()
    return bool(chunk) and tail["matcher"].feed(chunk)


def _submit(task: dict[str, Any]) -> dict[str, Any]:
    """Queue one task on the frame machine (or run its first ticks inline if
    it is already runnable -- it still finishes on the subscription)."""
    task.setdefault("state", "wait_start")
    task.setdefault("idle_since", None)
    task.setdefault("idle_deadline", None)
    _RUNTIME.tasks.append(task)
    _ensure_subscription()
    mode, value = task["start_mode"], task["start_value"]
    if mode != "now":
        print(f"[inject_scenarios] queued: {task['description']} "
              f"(start {mode}={value})")
    return task


def _on_update(_event: Any = None) -> None:
    now = time.monotonic()
    done: list[dict[str, Any]] = []
    for task in list(_RUNTIME.tasks):
        try:
            _advance(task, now)
        except Exception as exc:
            print(f"[inject_scenarios] task FAILED and was dropped "
                  f"({task.get('description')}): {exc}")
            task["state"] = "done"
        if task.get("state") == "done":
            done.append(task)
    for task in done:
        _RUNTIME.tasks.remove(task)


def _advance(task: dict[str, Any], now: float) -> None:
    state = task["state"]

    if state == "wait_start":
        mode, value = task["start_mode"], task["start_value"]
        if mode == "after_s":
            if task.get("start_deadline") is None:
                task["start_deadline"] = now + float(value)
            if now < task["start_deadline"]:
                return
        elif mode == "at_step":
            if task.get("step_tail") is None:
                task["step_tail"] = _make_step_tail(value)
            if not _step_tail_ready(task["step_tail"]):
                return
        task["state"] = "wait_idle" if task.get("idle_guard") else "run"
        return

    if state == "wait_idle":
        if task.get("idle_deadline") is None:
            task["idle_deadline"] = now + IDLE_TIMEOUT_S
        idle = _robot_is_idle()
        if idle is True:
            task["state"] = "run"
            return
        if now >= task["idle_deadline"]:
            why = ("joint speeds stayed above the idle bound"
                   if idle is False else "joint velocities could not be read")
            print(f"[inject_scenarios] WARNING: idle guard timed out after "
                  f"{IDLE_TIMEOUT_S:g} s ({why}); proceeding with "
                  f"{task['description']}.")
            task["state"] = "run"
        return

    if state == "run":
        # run_fn either finishes (returns "done") or hands over a per-frame
        # tick function (visit motion).
        outcome = task["run_fn"](task)
        task["state"] = "ticking" if outcome == "ticking" else "done"
        return

    if state == "ticking":
        if task["tick_fn"](task, now) == "done":
            task["state"] = "done"


# --- shared record/prim helpers ---------------------------------------------

def _write(record: dict[str, Any]) -> dict[str, Any]:
    session_dir = Path(record["session_dir"])
    ii.append_jsonl(session_dir / ii.GROUND_TRUTH_FILENAME, record)
    print(f"[inject_scenarios] {record['event_kind']}/{record['action']} "
          f"{record['name']} ep={record['episode_id']}")
    return record


def _register_prim(name: str, scenario: str, event_kind: str,
                   episode_id: str, prim_path: str) -> None:
    _RUNTIME.created[name] = {
        "scenario": scenario, "event_kind": event_kind,
        "episode_id": episode_id, "prim_path": prim_path,
    }


def _auto_name(prefix: str) -> str:
    name = f"{prefix}_{uuid.uuid4().hex[:6]}"
    ii.validate_name(name, ii._forbidden())
    return name


def _set_prim_position(prim: Any, position: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*position))
            return
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))


# ---------------------------------------------------------------------------
# Scenario 1: place_object  [foreign_object]
# ---------------------------------------------------------------------------

def place_object(
    position_xyz_world: Any = (0.45, -0.05, None),
    shape: str = "cube",
    size_m: float = 0.05,
    color_rgb: Any = (0.85, 0.15, 0.1),
    physics: bool = True,
    yaw_deg: float = 0.0,
    name: str | None = None,
    label: str = "",
    expected_impact: str = "benign",
    notes: str = "",
    start: str = "now",
    idle_guard: bool = True,
    min_edge_margin_px: float = MIN_EDGE_MARGIN_PX,
    allow_out_of_view: bool = False,
) -> dict[str, Any]:
    """A foreign object put on the bench and left there (never auto-removed).

    z=None in position_xyz_world means the resting default (bench + half the
    object + 5 mm). physics defaults to True; pass physics=False only when a
    non-settling, contact-inert object is what you want.
    """
    validate_scenario_impact(expected_impact)
    size, position, color = ii.validate_inject_args(
        shape, size_m, position_xyz_world, color_rgb, "benign"
    )
    check_view(position, "place_object", min_edge_margin_px, allow_out_of_view)
    name = name or _auto_name("foreign")
    ii.validate_name(name, ii._forbidden())
    mode, value = parse_start(start)
    episode_id = new_episode_id()
    trigger = start_trigger_text(start)

    def run(_task: dict[str, Any]) -> str:
        session_dir = ii.resolve_session_dir()
        path = ii._apply_inject(name, shape, size, position, float(yaw_deg),
                                color, bool(physics))
        _register_prim(name, "place_object", "foreign_object", episode_id, path)
        _write(scenario_record(
            scenario="place_object", event_kind="foreign_object",
            episode_id=episode_id, action="place", name=name, prim_path=path,
            session_dir=str(session_dir), trigger=trigger,
            position_xyz_world=position, shape=shape, size_m=size,
            yaw_deg=float(yaw_deg), color_rgb=color, physics=bool(physics),
            label=label, expected_impact=expected_impact, notes=notes,
        ))
        return "done"

    return _submit({
        "description": f"place_object {name}", "episode_id": episode_id,
        "start_mode": mode, "start_value": value, "idle_guard": idle_guard,
        "run_fn": run,
    })


# ---------------------------------------------------------------------------
# Scenario 2: visit  [transient_visitor]
# ---------------------------------------------------------------------------

def visit(
    shape: str = "hand",
    dwell_position: Any = (0.45, -0.10),
    dwell_s: float = 4.0,
    speed_mps: float = 0.3,
    entry_offset: Any | None = None,
    collide: bool = False,
    color_rgb: Any = (0.9, 0.75, 0.6),
    name: str | None = None,
    label: str = "",
    expected_impact: str = "benign",
    notes: str = "",
    start: str = "now",
    idle_guard: bool = True,
    min_edge_margin_px: float = MIN_EDGE_MARGIN_PX,
    allow_out_of_view: bool = False,
) -> dict[str, Any]:
    """Something enters the workspace, dwells, and leaves (then its prim is
    removed). Kinematic, collision OFF by default: the robot and the parts
    are never touched unless collide=True is asked for.

    Records at: enter_start, in_view (first frame the projected centre is in
    view), dwell_start, leave_start, out_of_view, removed.
    """
    validate_scenario_impact(expected_impact)
    if not (float(speed_mps) > 0):
        raise ValueError("speed_mps must be > 0.")
    geometry = visit_geometry(shape, dwell_position, entry_offset)
    check_view(geometry["dwell"], f"visit({shape}) dwell position",
               min_edge_margin_px, allow_out_of_view)
    name = name or _auto_name(f"visitor_{shape}")
    ii.validate_name(name, ii._forbidden())
    color = ii._xyz(color_rgb, "color_rgb")
    mode, value = parse_start(start)
    episode_id = new_episode_id()
    trigger = start_trigger_text(start)

    common = dict(scenario="visit", event_kind="transient_visitor",
                  episode_id=episode_id, name=name, trigger=trigger,
                  shape=shape, label=label, expected_impact=expected_impact,
                  notes=notes)
    extras = {
        "dwell_position_world": list(geometry["dwell"]),
        "entry_offset_world": list(geometry["entry_offset"]),
        "dwell_s": float(dwell_s), "speed_mps": float(speed_mps),
        "collide": bool(collide),
    }

    def record_phase(action: str, position: tuple[float, float, float],
                     prim_path: str) -> None:
        _write(scenario_record(
            **common, action=action, prim_path=prim_path,
            session_dir=str(ii.resolve_session_dir()),
            position_xyz_world=position, physics=bool(collide), extra=extras,
        ))

    def run(task: dict[str, Any]) -> str:
        prim_path = _create_visitor_prim(name, geometry, color, bool(collide))
        _register_prim(name, "visit", "transient_visitor", episode_id, prim_path)
        task.update(
            name=name, prim_path=prim_path, position=list(geometry["start"]),
            target=list(geometry["dwell"]), enter_start=list(geometry["start"]),
            phase="enter",
            was_in_view=ii.camera_view_fields(geometry["start"])["in_camera_view"],
            dwell_until=None, last_tick=time.monotonic(),
            record_phase=record_phase,
        )
        record_phase("enter_start", tuple(task["position"]), prim_path)
        if task["was_in_view"]:
            record_phase("in_view", tuple(task["position"]), prim_path)
        task["tick_fn"] = _visit_tick
        return "ticking"

    return _submit({
        "description": f"visit {shape} {name}", "episode_id": episode_id,
        "start_mode": mode, "start_value": value, "idle_guard": idle_guard,
        "run_fn": run, "dwell_s": float(dwell_s), "speed_mps": float(speed_mps),
    })


def _create_visitor_prim(
    name: str, geometry: dict[str, Any],
    color: tuple[float, float, float], collide: bool,
) -> str:
    from pxr import Gf, UsdGeom

    stage = ii._stage()
    ii._ensure_root(stage)
    path = ii._prim_path(name)
    if stage.GetPrimAtPath(path):
        raise ValueError(f"{path} already exists.")
    if geometry["shape"] == "hand":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.GetSizeAttr().Set(1.0)
        scale = geometry["size"]["box_xyz"]
    else:
        geom = UsdGeom.Capsule.Define(stage, path)
        geom.GetRadiusAttr().Set(geometry["size"]["radius"])
        geom.GetHeightAttr().Set(geometry["size"]["height"])
        geom.GetAxisAttr().Set("Z")
        scale = None
    prim = geom.GetPrim()
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*geometry["start"]))
    if scale is not None:
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    if collide:
        from pxr import Sdf, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        prim.CreateAttribute(
            "physics:kinematicEnabled", Sdf.ValueTypeNames.Bool
        ).Set(True)
    return path


def _visit_tick(task: dict[str, Any], now: float) -> str | None:
    dt = max(0.0, min(now - task["last_tick"], 0.1))
    task["last_tick"] = now
    stage = ii._stage()
    prim = stage.GetPrimAtPath(task["prim_path"])
    if not prim:
        print(f"[inject_scenarios] visitor prim vanished; ending {task['description']}")
        return "done"
    phase = task["phase"]
    record = task["record_phase"]

    if phase == "dwell":
        if now >= task["dwell_until"]:
            task["phase"] = "leave"
            task["target"] = list(task["leave_target"])
            record("leave_start", tuple(task["position"]), task["prim_path"])
        return None

    # enter or leave: straight line toward target at speed_mps.
    position, target = task["position"], task["target"]
    delta = [target[i] - position[i] for i in range(3)]
    distance = sum(d * d for d in delta) ** 0.5
    step = task["speed_mps"] * dt
    if distance <= max(step, 1e-6):
        new_position = list(target)
    else:
        new_position = [position[i] + delta[i] * step / distance for i in range(3)]
    task["position"] = new_position
    _set_prim_position(prim, tuple(new_position))

    in_view = ii.camera_view_fields(new_position)["in_camera_view"]
    if in_view and not task["was_in_view"]:
        record("in_view", tuple(new_position), task["prim_path"])
    if (not in_view) and task["was_in_view"] and phase == "leave":
        record("out_of_view", tuple(new_position), task["prim_path"])
    task["was_in_view"] = in_view

    if distance <= max(step, 1e-6):  # arrived
        if phase == "enter":
            task["phase"] = "dwell"
            task["dwell_until"] = now + task["dwell_s"]
            # it leaves back the way it came in
            task["leave_target"] = list(task["enter_start"])
            record("dwell_start", tuple(new_position), task["prim_path"])
        else:  # leave arrived at its exit point
            last = tuple(task["position"])
            stage.RemovePrim(task["prim_path"])
            _RUNTIME.created.pop(task.get("name", ""), None)
            record("removed", last, task["prim_path"])
            return "done"
    return None


# ---------------------------------------------------------------------------
# Scenario 3: nudge_object  [external_displacement]
# ---------------------------------------------------------------------------

def _registry_prim_path(object_id: str) -> str:
    """Resolve a REGISTERED object's prim path from the registry + the stage.

    The registry gives the object's tf_frame, which is the scene prim's name
    (model_registry.yaml documents that identity); the live stage is searched
    for that prim rather than assuming /World/<name>, so children of a
    fixture resolve too. Never hardcoded."""
    import yaml

    with open(ii._REGISTRY_PATH, encoding="utf-8") as handle:
        registry = yaml.safe_load(handle) or {}
    entry = (registry.get("objects") or {}).get(object_id)
    if not isinstance(entry, dict):
        known = sorted((registry.get("objects") or {}))
        raise ValueError(f"{object_id!r} is not in the registry. Objects: {known}")
    frame = entry.get("tf_frame") or object_id
    stage = ii._stage()
    direct = stage.GetPrimAtPath(f"/World/{frame}")
    if direct:
        return str(direct.GetPath())
    for prim in stage.Traverse():
        if prim.GetName() == frame:
            return str(prim.GetPath())
    raise RuntimeError(f"no prim named {frame!r} found on the stage.")


def _prim_world_pose(prim: Any) -> dict[str, Any]:
    from pxr import Usd, UsdGeom

    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    translation = matrix.ExtractTranslation()
    quat = matrix.ExtractRotationQuat()
    imaginary = quat.GetImaginary()
    return {
        "position": [float(translation[i]) for i in range(3)],
        "orientation_xyzw": [float(imaginary[0]), float(imaginary[1]),
                             float(imaginary[2]), float(quat.GetReal())],
    }


def nudge_object(
    object_id: str,
    delta_xyz: Any,
    yaw_deg: float = 0.0,
    label: str = "",
    expected_impact: str = "violation",
    notes: str = "",
    start: str = "now",
    idle_guard: bool = True,
) -> dict[str, Any]:
    """Displace an EXISTING registered scene object, simulating an external
    cause (not the robot). The object IS in TF, so the agents can see its
    pose move -- that visibility is expected, and the leak audit classifies
    these records as expected_visible, never as a leak.
    """
    validate_scenario_impact(expected_impact)
    delta = ii._xyz(delta_xyz, "delta_xyz")
    mode, value = parse_start(start)
    episode_id = new_episode_id()
    trigger = start_trigger_text(start)

    def run(_task: dict[str, Any]) -> str:
        from pxr import Gf, UsdGeom

        session_dir = ii.resolve_session_dir()
        prim_path = _registry_prim_path(object_id)
        stage = ii._stage()
        prim = stage.GetPrimAtPath(prim_path)
        pre = _prim_world_pose(prim)
        xform = UsdGeom.Xformable(prim)
        translate_op = None
        rotate_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                rotate_op = op
        if translate_op is None:
            translate_op = xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(*pre["position"]))
        current = translate_op.Get()
        translate_op.Set(Gf.Vec3d(current[0] + delta[0],
                                  current[1] + delta[1],
                                  current[2] + delta[2]))
        if yaw_deg:
            if rotate_op is None:
                rotate_op = xform.AddRotateZOp()
                rotate_op.Set(0.0)
            rotate_op.Set(float(rotate_op.Get() or 0.0) + float(yaw_deg))
        post = _prim_world_pose(prim)
        _write(scenario_record(
            scenario="nudge_object", event_kind="external_displacement",
            episode_id=episode_id, action="nudge", name=object_id,
            prim_path=prim_path, session_dir=str(session_dir), trigger=trigger,
            position_xyz_world=tuple(post["position"]),
            yaw_deg=float(yaw_deg), label=label,
            expected_impact=expected_impact, notes=notes,
            extra={"pre_pose_world": pre, "post_pose_world": post,
                   "delta_xyz_world": list(delta)},
        ))
        return "done"

    return _submit({
        "description": f"nudge_object {object_id}", "episode_id": episode_id,
        "start_mode": mode, "start_value": value, "idle_guard": idle_guard,
        "run_fn": run,
    })


# ---------------------------------------------------------------------------
# Scenario 4: remove + cancel_all  [removal]
# ---------------------------------------------------------------------------

def remove(
    name: str,
    label: str = "",
    notes: str = "",
    start: str = "now",
    idle_guard: bool = True,
) -> dict[str, Any]:
    """Remove a prim this module (or inject_intruder) created, recording its
    LAST position before removal."""
    mode, value = parse_start(start)
    meta = _RUNTIME.created.get(name, {})
    episode_id = meta.get("episode_id") or new_episode_id()
    trigger = start_trigger_text(start)

    def run(_task: dict[str, Any]) -> str:
        session_dir = ii.resolve_session_dir()
        path, last_position = ii._apply_remove(name)
        _RUNTIME.created.pop(name, None)
        _write(scenario_record(
            scenario="remove", event_kind="removal", episode_id=episode_id,
            action="removed", name=name, prim_path=path,
            session_dir=str(session_dir), trigger=trigger,
            position_xyz_world=last_position, label=label, notes=notes,
        ))
        return "done"

    return _submit({
        "description": f"remove {name}", "episode_id": episode_id,
        "start_mode": mode, "start_value": value, "idle_guard": idle_guard,
        "run_fn": run,
    })


def cancel_all() -> None:
    """Stop every running callback and remove every prim this module created,
    writing one record per removal (and per cancelled pending task)."""
    pending = list(_RUNTIME.tasks)
    _RUNTIME.tasks.clear()
    try:
        session_dir = str(ii.resolve_session_dir())
    except RuntimeError as exc:
        session_dir = None
        print(f"[inject_scenarios] cancel_all: no session dir ({exc}); "
              "removing prims without records.")
    for task in pending:
        if session_dir and task.get("prim_path") is None:
            _write(scenario_record(
                scenario="cancel_all", event_kind="removal",
                episode_id=task.get("episode_id") or new_episode_id(),
                action="cancelled", name=task.get("description", "?"),
                prim_path="", session_dir=session_dir,
                notes="pending task cancelled before it created a prim",
            ))
    stage = ii._stage()
    for name, meta in list(_RUNTIME.created.items()):
        prim = stage.GetPrimAtPath(meta["prim_path"])
        last_position = ii.prim_world_position(prim) if prim else None
        if prim:
            stage.RemovePrim(meta["prim_path"])
        _RUNTIME.created.pop(name, None)
        if session_dir:
            _write(scenario_record(
                scenario="cancel_all", event_kind="removal",
                episode_id=meta["episode_id"], action="removed", name=name,
                prim_path=meta["prim_path"], session_dir=session_dir,
                position_xyz_world=last_position,
                notes="removed by cancel_all",
            ))
    print(f"[inject_scenarios] cancel_all: {len(pending)} task(s) stopped, "
          "created prims removed.")


__all__ = [
    "EVENT_KINDS", "MIN_EDGE_MARGIN_PX", "PRESETS", "SCENARIO_IMPACTS",
    "VISIT_PHASES", "VISIT_SHAPES", "ViewRefusal", "cancel_all", "check_view",
    "new_episode_id", "nudge_object", "parse_start", "place_object",
    "presets", "remove", "scenario_record", "start_trigger_text",
    "visit", "visit_geometry",
]
