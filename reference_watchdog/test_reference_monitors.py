"""Offline proof that the reference monitors detect every injected event kind.

Runs the two sources from monitors.py through the REAL authored-monitor
sandbox (robot_backend.watchdog.authored.AuthoredMonitor: validator, line
budget, restricted builtins, provided numpy) against a synthetic depth camera
that uses the cell's real intrinsics and the real camera extrinsics from
sim_moveit_bringup/config/cameras.yaml. The scene is rendered by ray/plane
intersection so ctx.deproject() and the monitor's bench-plane model are
checked against one another, not against themselves.

Run with the workspace recipe (see docs/dev or memory reference_test_recipe):
  cd ros2_ws && unset VIRTUAL_ENV && source /opt/ros/jazzy/setup.bash \
    && source install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    PYTHONPATH="$PWD/src/robot_backend:$PYTHONPATH" \
    .venv/bin/python3 -m pytest -q ../reference_watchdog/test_reference_monitors.py
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_backend.watchdog.authored import (  # noqa: E402
    MAX_SOURCE_CHARS,
    AuthoredMonitor,
    validate_source,
)
from robot_backend.watchdog.engine import ARRAY_WALL_BUDGET_MS, MonitorEngine  # noqa: E402

import monitors as ref  # noqa: E402

# Real cell camera: 640x360, 60 deg HFOV (K from camera_info), extrinsics from
# sim_moveit_bringup/config/cameras.yaml.
W, H = 640, 360
FX = FY = 554.2563
CX, CY = 320.0, 180.0
CAM_T = np.array([0.25, -0.2, 1.7])
CAM_Q = [0.0, 0.99863, -0.05234, 0.0]
BENCH_Z = 0.01


def quat_to_rot(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


ROT = quat_to_rot(CAM_Q)
_v, _u = np.mgrid[0:H, 0:W]
_DIRS = np.stack([(_u - CX) / FX, (_v - CY) / FY, np.ones((H, W))], axis=-1) @ ROT.T


class Scene:
    """Horizontal top faces over a bench plane. boxes: name -> (x0,x1,y0,y1,top_z)."""

    # the bench does not fill the view: past its edges the camera sees the
    # floor, far below the bench plane (top-left and top-right of the real image)
    BENCH_XY = (-0.65, 1.05, -0.84, 0.25)
    FLOOR_Z = -0.74

    def __init__(self):
        self.boxes: dict[str, tuple[float, float, float, float, float]] = {}

    def square(self, name, x, y, side, top_z):
        self.boxes[name] = (x - side / 2, x + side / 2, y - side / 2, y + side / 2, top_z)

    def rect(self, name, x0, x1, y0, y1, top_z):
        self.boxes[name] = (x0, x1, y0, y1, top_z)

    def remove(self, name):
        self.boxes.pop(name, None)

    def render(self) -> np.ndarray:
        dz = _DIRS[..., 2]
        depth = (BENCH_Z - CAM_T[2]) / dz
        bx = CAM_T[0] + depth * _DIRS[..., 0]
        by = CAM_T[1] + depth * _DIRS[..., 1]
        x0, x1, y0, y1 = self.BENCH_XY
        off_bench = (bx < x0) | (bx > x1) | (by < y0) | (by > y1)
        depth = np.where(off_bench, (self.FLOOR_Z - CAM_T[2]) / dz, depth)
        for (x0, x1, y0, y1, top) in self.boxes.values():
            s = (top - CAM_T[2]) / dz
            px = CAM_T[0] + s * _DIRS[..., 0]
            py = CAM_T[1] + s * _DIRS[..., 1]
            inside = (px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)
            depth = np.where(inside & (s < depth), s, depth)
        out = depth.astype(np.float32)
        out.flags.writeable = False
        return out


class FakeStreams:
    def __init__(self, scene: Scene):
        self.scene = scene

    def array_snapshot(self):
        from robot_backend.watchdog.timebase import wall_ns
        now = wall_ns()
        return {
            "taken_wall_ns": now, "rgb": None,
            "depth": (self.scene.render(), now),
            "camera": {
                "width": W, "height": H,
                "K": [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0],
                "frame_id": "camera_depth_optical_frame", "D": [], "distortion_model": "",
                "extrinsics_from_yaml": {"xyz": list(CAM_T), "quat_xyzw": list(CAM_Q)},
            },
        }


class FakeRecorder:
    """Registry objects + the tcp cell frame, poses mutable by the test."""

    def __init__(self):
        self.objects: dict[str, dict[str, float]] = {}
        self.tcp = {"x": 0.6, "y": -0.5, "z": 0.25}
        self.links: dict[str, tuple[float, float, float]] = {}

    def object_ids(self):
        return list(self.objects)

    def frame_ids(self):
        return ["tcp"]

    def _sample(self, key, ident, pos):
        from robot_backend.watchdog.timebase import wall_ns
        return {key: ident, "wall_ns": wall_ns(), "tf_stamp_ns": 0,
                "pose": {"position": dict(pos),
                         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}

    def latest_object_pose(self, object_id):
        pos = self.objects.get(object_id)
        return None if pos is None else self._sample("object_id", object_id, pos)

    def latest_frame_pose(self, frame_id):
        return self._sample("frame_id", "tcp", self.tcp) if frame_id == "tcp" else None

    def latest_joint_state(self):
        return None

    def lookup_transform(self, parent, child):
        if parent != "world" or child not in self.links:
            raise RuntimeError(f"no transform {parent}->{child} in this fake")
        x, y, z = self.links[child]
        return {"translation": {"x": x, "y": y, "z": z},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}

    def get_joint_window(self, *a):
        return []

    def get_object_pose_window(self, *a):
        return []

    def get_frame_window(self, *a):
        return []


class FakeBus:
    def __init__(self):
        self.events = []

    def publish(self, event):
        self.events.append(event)


class Cell:
    """One synthetic cell: scene geometry + registry poses kept in step."""

    def __init__(self):
        self.scene = Scene()
        self.rec = FakeRecorder()
        self.streams = FakeStreams(self.scene)
        self.engine = MonitorEngine(FakeBus())
        self.wall_ms: list[float] = []
        # gantry rail across the top of the image (static, unregistered, high)
        self.scene.rect("gantry", -1.0, 1.5, -0.86, -0.76, 0.50)
        # the robot's own base column, top-left of the image as in the real
        # cell (world x large, y negative), unregistered and high
        self.scene.square("robot_base", 0.95, -0.65, 0.16, 0.20)
        # registry parts and fixture
        self.put_object("base1", 0.79, -0.20, 0.20, 0.03)
        self.put_object("u_orange", 0.45, -0.15, 0.08, 0.04)
        self.put_object("u_brown", 0.20, -0.55, 0.08, 0.04)
        self.set_tcp(0.6, -0.55)

    def put_object(self, name, x, y, side, top_z):
        self.scene.square(name, x, y, side, top_z)
        self.rec.objects[name] = {"x": x, "y": y, "z": top_z / 2}

    BASE = (0.95, -0.65)

    def set_tcp(self, x, y, z=0.25):
        """Arm geometry AND the TF chain the monitor reads, kept consistent:
        base column, an upper arm along x from the base to the elbow above
        the tcp's x, a forearm along y down to the tcp, a wrist column."""
        self.rec.tcp = {"x": x, "y": y, "z": z}
        bx, by = self.BASE
        self.scene.rect("arm_upper", min(bx, x) - 0.05, max(bx, x) + 0.05, by - 0.05, by + 0.05, 0.45)
        self.scene.rect("arm_fore", x - 0.05, x + 0.05, min(by, y) - 0.05, max(by, y) + 0.05, 0.45)
        self.scene.square("arm_column", x, y, 0.14, 0.45)
        # elbow joint housing: offset from the link-origin skeleton, high
        self.scene.square("arm_elbow", x + 0.15, by, 0.10, 0.67)
        self.rec.links = {
            "base_link": (bx, by, 0.0), "shoulder_link": (bx, by, 0.45),
            "upper_arm_link": (bx, by, 0.45), "forearm_link": (x, by, 0.45),
            "wrist_1_link": (x, y, 0.45), "wrist_2_link": (x, y, 0.35),
            "wrist_3_link": (x, y, 0.30), "tcp": (x, y, z),
        }

    def arm(self, spec) -> AuthoredMonitor:
        kw = dict(spec)
        kw.pop("monitor_id", None)
        kw.pop("clause", None)
        kw.pop("halt_on_fire", None)
        return AuthoredMonitor(
            source=kw.pop("code"), recorder=self.rec, engine=self.engine,
            camera=self.streams, **kw,
        )

    def tick(self, monitor: AuthoredMonitor):
        value = monitor.evaluate(None)
        if monitor.last_wall_ms is not None:
            self.wall_ms.append(monitor.last_wall_ms)
        if value is None:
            return None
        return json.loads(monitor.last_detail)

    def run_until_fire(self, monitor, max_ticks):
        for i in range(max_ticks):
            fire = self.tick(monitor)
            if fire is not None:
                return i + 1, fire
        return None, None


# ---------------------------------------------------------------------------

def test_sources_pass_the_sandbox_validator_and_size_limit():
    for src in (ref.SCENE_WATCH_SOURCE, ref.UNDISTURBED_WATCH_SOURCE):
        assert len(src) <= MAX_SOURCE_CHARS, len(src)
        validate_source(src)


def test_quiet_scene_and_arm_sweep_never_fire():
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(4):
        assert cell.tick(mon) is None
    # the arm sweeps across the whole bench, over both parts
    for x in np.linspace(0.9, -0.3, 9):
        cell.set_tcp(float(x), -0.25)
        assert cell.tick(mon) is None, f"false fire with tcp at x={x:.2f}"
    for _ in range(3):
        assert cell.tick(mon) is None


def test_foreign_cube_is_detected_with_position_height_and_first_seen():
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    from robot_backend.watchdog.timebase import wall_ns
    t_place = wall_ns()
    cell.scene.square("cube", 0.08, -0.38, 0.05, BENCH_Z + 0.05)  # the S1-S4 intruder
    ticks, fire = cell.run_until_fire(mon, 6)
    assert fire is not None, "cube never reported"
    assert fire["kind"] == "new_static"
    assert abs(fire["x"] - 0.08) < 0.03 and abs(fire["y"] + 0.38) < 0.03, fire
    assert 0.03 < fire["height_m"] < 0.07, fire
    assert 0.03 < fire["side_m"] < 0.08, fire
    assert fire["since_wall_ns"] >= t_place - 1_000_000
    assert ticks <= 4, ticks
    # once reported it is part of the reference: no re-report
    for _ in range(4):
        assert cell.tick(mon) is None
    # the arm hovers over the cube and leaves: still no re-report
    cell.set_tcp(0.08, -0.38)
    assert cell.tick(mon) is None
    cell.set_tcp(0.6, -0.55)
    for _ in range(4):
        assert cell.tick(mon) is None


def test_cube_under_the_arm_at_arm_time_is_found_when_revealed():
    cell = Cell()
    cell.set_tcp(0.08, -0.38)  # arm parked over where the cube will be
    cell.scene.square("cube", 0.08, -0.38, 0.05, BENCH_Z + 0.05)
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    cell.set_tcp(0.6, -0.55)
    ticks, fire = cell.run_until_fire(mon, 6)
    assert fire is not None and fire["kind"] == "new_static", fire
    assert abs(fire["x"] - 0.08) < 0.03 and abs(fire["y"] + 0.38) < 0.03, fire


def test_robot_structure_uncovered_by_the_arm_is_not_reported():
    """Live 2026-10-23 false fire: the arm lifted off a low static structure
    next to the robot base at run start and the uncovered strip was reported
    as a new object. Structure uncovered next to the robot's region is the
    robot's own; a cube uncovered clear of the robot still is reported."""
    cell = Cell()
    # the base flange: low, flush against the robot base column, and under the
    # arm at arm time (the arm starts folded over its own base, as live)
    cell.scene.rect("base_flange", 0.80, 0.87, -0.70, -0.60, BENCH_Z + 0.03)
    cell.set_tcp(0.84, -0.55)
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    cell.set_tcp(0.2, -0.3)  # arm swings away: the plate is uncovered
    for _ in range(6):
        assert cell.tick(mon) is None


def test_floor_past_the_bench_edge_uncovered_by_the_arm_is_not_reported():
    """Live 2026-10-23 false fire #2: the arm started folded over the bench
    corner; when it moved, the floor beyond the bench edge (far BELOW the
    plane) was uncovered and reported as 'removed' with NaN coordinates."""
    cell = Cell()
    cell.set_tcp(1.0, -0.75)  # column + link cover the bench corner and the floor past it
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    cell.set_tcp(0.2, -0.3)
    for _ in range(8):
        fire = cell.tick(mon)
        assert fire is None, fire
    # and a cube placed later near that corner (clear of the raised upper
    # arm's projected footprint) is still reported, with finite values
    cell.scene.square("cube", 0.90, -0.35, 0.05, BENCH_Z + 0.05)
    ticks, fire = cell.run_until_fire(mon, 6)
    assert fire is not None and fire["kind"] == "new_static", fire
    assert all(math.isfinite(fire[k]) for k in ("x", "y", "z", "height_m")), fire


def test_uncovered_structure_in_the_gantry_strip_is_not_reported():
    """Live 2026-10-24: the end of the energy chain at y=-0.75, uncovered
    when the arm left home, fired once. The rail strip is robot territory."""
    cell = Cell()
    cell.scene.square("chain_end", 0.60, -0.76, 0.07, BENCH_Z + 0.05)
    cell.set_tcp(0.60, -0.70)  # arm parked over it at arm time
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    cell.set_tcp(0.2, -0.3)
    for _ in range(8):
        fire = cell.tick(mon)
        assert fire is None, fire


def test_real_arm_pose_from_the_bag_leaves_the_cube_spot_unmasked():
    """Live 2026-10-24 session 20261024T011010Z: the cube at (0.08,-0.38)
    was never reported because the near-vertical upper arm's tube, sized at
    the elbow's depth, covered its pixel all run. Link positions below are
    the bag's at 00:58:47 of the previous session (same home-ish pose)."""
    from robot_backend.watchdog.authored import compile_check
    ns: dict = {}
    import types
    code = compile_check(ref.SCENE_WATCH_SOURCE, {"np": np})
    sandbox = code.__globals__
    cell = Cell()
    cell.rec.links = {
        "base_link": (0.23, -0.65, 0.09), "shoulder_link": (0.23, -0.65, 0.25),
        "upper_arm_link": (0.23, -0.65, 0.25), "forearm_link": (0.26, -0.63, 0.67),
        "wrist_1_link": (0.53, -0.38, 0.49), "wrist_2_link": (0.62, -0.34, 0.49),
        "wrist_3_link": (0.62, -0.34, 0.39), "tcp": (0.62, -0.34, 0.24),
    }
    mon = cell.arm(ref.scene_watch_spec())  # runs setup() once via the first call
    ctx = mon.ctx
    shape = ctx.state["bdepth"].shape
    hgt = np.zeros(shape, dtype=np.float32)
    mask = sandbox["rmask"](ctx, hgt, np.ones(shape, dtype=bool))
    cube = (int(179.5 // 4), int(377.5 // 4))
    assert not mask[cube], "cube pixel is inside the arm mask"
    # and the arm itself is masked: elbow and tcp pixels
    for name in ("forearm_link", "wrist_1_link", "tcp"):
        v, u, _ = sandbox["cpt"](ctx, {"x": cell.rec.links[name][0], "y": cell.rec.links[name][1],
                                       "z": cell.rec.links[name][2]})
        assert mask[int(v), int(u)], name
    # margin: the cube must be clear by at least 4 cells (16 px) in every direction
    v0, u0 = cube
    assert not mask[v0 - 4:v0 + 5, u0 - 4:u0 + 5].any(), "cube spot too close to the mask edge"


def test_registered_part_placed_elsewhere_is_explained_not_reported():
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    # executor carries u_orange to a new spot: tf follows, so the scene watch
    # must fold it into the reference silently
    cell.set_tcp(0.45, -0.15)
    assert cell.tick(mon) is None
    cell.put_object("u_orange", 0.30, -0.40, 0.08, 0.04)
    cell.set_tcp(0.30, -0.40)
    assert cell.tick(mon) is None
    cell.set_tcp(0.6, -0.55)
    for _ in range(6):
        assert cell.tick(mon) is None


def _hand(cell, x, y):
    # inject_scenarios "hand": 0.40 x 0.08 x 0.08 m box at bench + 0.15 m
    cell.scene.rect("hand", x - 0.20, x + 0.20, y - 0.04, y + 0.04, BENCH_Z + 0.15 + 0.04)


def test_hand_visit_is_reported_on_entry_and_on_leaving():
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    # enters from the far x edge at 0.3 m/s (one tick = 1 s), dwell at (0.45,-0.10)
    xs = [1.35, 1.05, 0.75, 0.45]
    entry_fire = None
    for i, x in enumerate(xs):
        _hand(cell, x, -0.10)
        fire = cell.tick(mon)
        if fire is not None:
            entry_fire = (i + 1, fire)
            break
    assert entry_fire is not None, "hand entry never reported"
    _, fire = entry_fire
    assert fire["kind"] in ("moving", "new_static"), fire
    assert fire["height_m"] > 0.10, fire
    # re-arm as the client does after a fire: fresh reference with the hand in it
    _hand(cell, 0.45, -0.10)
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(4):  # dwell
        assert cell.tick(mon) is None
    leave_fire = None
    for i, x in enumerate([0.75, 1.05, 1.35]):
        _hand(cell, x, -0.10)
        fire = cell.tick(mon)
        if fire is not None:
            leave_fire = (i + 1, fire)
            break
    if leave_fire is None:
        cell.scene.remove("hand")
        ticks, fire = cell.run_until_fire(mon, 4)
        assert fire is not None, "hand leaving never reported"
        leave_fire = (ticks, fire)
    assert leave_fire[1]["kind"] in ("moving", "removed"), leave_fire


def test_hand_crossing_the_working_arm_is_still_reported():
    """Live 2026-10-24: the hand visit ran while the arm worked at bench
    level and was never reported. With the geometric arm mask, the hand is
    masked only where it overlaps the arm's own silhouette."""
    cell = Cell()
    cell.set_tcp(0.30, -0.15)  # arm down over the bench, in the hand's path
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    fired = None
    for i, x in enumerate([-0.45, -0.15, 0.15, 0.45]):  # enters from the far side, as live
        _hand(cell, x, -0.10)
        fire = cell.tick(mon)
        if fire is not None:
            fired = (i + 1, fire)
            break
    assert fired is not None, "hand never reported while the arm worked"
    assert fired[1]["height_m"] > 0.10, fired


def test_gripper_down_at_bench_level_never_unmasks_the_arm():
    """Live 2026-10-24: with the gripper at bench level the old mask lost the
    arm entirely and fired on it. The arm tube must hold at any tcp height."""
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    for x, y, z in [(0.5, -0.3, 0.25), (0.45, -0.15, 0.06), (0.45, -0.15, 0.03),
                    (0.45, -0.15, 0.25), (0.2, -0.45, 0.04), (0.2, -0.45, 0.30)]:
        cell.set_tcp(x, y, z)
        fire = cell.tick(mon)
        assert fire is None, (x, y, z, fire)


def test_bench_re_exposed_behind_a_moving_hand_is_not_a_moving_body():
    """Live 2026-10-25: two 'moving' reports with NEGATIVE height trailed the
    hand -- the bench coming back into view where the hand had been. Only a
    surface closer than the reference is a moving body."""
    cell = Cell()
    _hand(cell, 0.45, -0.10)            # hand present at arm time (as after a re-arm)
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    for x in (0.75, 1.05, 1.35):        # it leaves: only 'removed' (or a positive-height
        _hand(cell, x, -0.10)           # 'moving' of the hand itself) may fire
        fire = cell.tick(mon)
        if fire is not None:
            assert fire["kind"] in ("removed", "moving"), fire
            if fire["kind"] == "moving":
                assert fire["height_m"] > 0.10, fire
    cell.scene.remove("hand")
    for _ in range(4):
        fire = cell.tick(mon)
        assert fire is None or fire["kind"] == "removed", fire


def test_undisturbed_watch_ignores_registry_frames():
    """Placement/reorient/hole frames ride on the fixtures and would double
    every fixture report (live 2026-10-25: u_brown_placement reported)."""
    cell = Cell()
    cell.rec.objects["u_brown_placement"] = {"x": 0.49, "y": 0.07, "z": 0.03}
    mon = cell.arm(ref.undisturbed_watch_spec())
    for _ in range(2):
        assert cell.tick(mon) is None
    cell.rec.objects["u_brown_placement"] = {"x": 0.55, "y": 0.07, "z": 0.03}
    for _ in range(3):
        assert cell.tick(mon) is None


def test_person_at_bench_edge_is_reported():
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    # inject_scenarios "person": capsule r 0.18 h 1.0 at the bench edge; top face only
    fired = None
    for y in (0.40, 0.15, -0.10, -0.45, -0.45):
        cell.scene.square("person", 0.10, y, 0.36, 1.0)
        fire = cell.tick(mon)
        if fire is not None:
            fired = fire
            break
    assert fired is not None, "person never reported"
    assert fired["height_m"] > 0.5, fired


def test_undisturbed_watch_reports_a_nudge_but_follows_the_robot():
    cell = Cell()
    mon = cell.arm(ref.undisturbed_watch_spec())
    for _ in range(3):
        assert cell.tick(mon) is None
    # robot picks u_orange up and carries it: tcp is on it, reference follows
    cell.set_tcp(0.45, -0.15)
    cell.rec.objects["u_orange"] = {"x": 0.45, "y": -0.15, "z": 0.20}
    assert cell.tick(mon) is None
    cell.set_tcp(0.30, -0.40)
    cell.rec.objects["u_orange"] = {"x": 0.30, "y": -0.40, "z": 0.02}
    assert cell.tick(mon) is None
    cell.set_tcp(0.6, -0.55)
    for _ in range(3):
        assert cell.tick(mon) is None
    # external nudge: 3 cm while the tcp is far away (inject_scenarios preset)
    cell.rec.objects["u_orange"] = {"x": 0.33, "y": -0.40, "z": 0.02}
    fire = cell.tick(mon)
    assert fire is not None and fire["kind"] == "object_moved", fire
    assert fire["object_id"] == "u_orange" and abs(fire["delta_m"] - 0.03) < 0.005, fire
    # jitter below tolerance never fires
    cell.rec.objects["u_brown"] = {"x": 0.204, "y": -0.55, "z": 0.02}
    for _ in range(3):
        assert cell.tick(mon) is None


def test_every_array_call_is_inside_the_wall_budget():
    cell = Cell()
    mon = cell.arm(ref.scene_watch_spec())
    for x in np.linspace(0.9, -0.3, 6):
        cell.set_tcp(float(x), -0.25)
        cell.tick(mon)
    cell.scene.square("cube", 0.08, -0.38, 0.05, BENCH_Z + 0.05)
    _hand(cell, 0.75, -0.10)
    for _ in range(6):
        cell.tick(mon)
    assert cell.wall_ms, "no calls measured"
    worst = max(cell.wall_ms)
    print(f"\nscene watch wall ms: max {worst:.1f}, mean {np.mean(cell.wall_ms):.1f}")
    assert worst < ARRAY_WALL_BUDGET_MS * 0.5, worst


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q", "-s"]))
