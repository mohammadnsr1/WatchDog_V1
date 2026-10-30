"""Step 2: replay_state.npz + config -> rendered frames (runs INSIDE Isaac Sim 5.1).

Script Editor (uses the running app; see README for the full snippet):

    import sys, importlib, asyncio
    sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1"); importlib.invalidate_caches()
    import paper_media.isaac_replay.render_replay as rr; importlib.reload(rr)
    asyncio.ensure_future(rr.run_async("<session>/replay/replay_state.npz",
                                       "paper_media/isaac_replay/replay_config.yaml", check=True))

Standalone (Isaac's Python):

    ~/isaacsim/python.sh paper_media/isaac_replay/render_replay.py \
        --state <session>/replay/replay_state.npz --config <config.yaml> [--check] [--overwrite]

Physics-free in effect (see README):
  * The stage is opened, its layers are locked against saving, and every edit
    goes to the in-memory session layer. Nothing is ever saved.
  * Gravity is set to zero, every collision is disabled, every part's rigid
    body is kinematic, and configured graphs (ROS bridge, joint command
    subscriber, TF publisher) are deactivated.
  * Each frame overwrites the whole robot state: joint positions, zero joint
    velocities, and drive targets equal to the positions. One physics step
    then commits it, and Replicator renders `warmup_subframes` with the
    timeline held (delta_time 0) before the capture.

Time: each frame's wall_time_iso and log_time_ns come straight from the npz
(the bag's log time). header.stamp and timestamp_ns are never used.
Quaternions in the npz are wxyz, which is also isaacsim.core's order.
"""
from __future__ import annotations

import json
import math
import struct
import sys
import time
import zlib
from pathlib import Path
from typing import Any, Optional

import numpy as np

DEFAULTS: dict[str, Any] = {
    "stage": None,
    "robot_prim": "/World/ur5e",
    "articulation_root": "/World/ur5e/root_joint",
    "world_frame": "world",
    "world_aliases": [],
    "part_map": {},
    "part_physics": "kinematic",          # kinematic | disable
    "deactivate_prims": [],
    "mimic": {"source": "usd", "convention": "physx", "urdf_path": None, "offset_units": "deg"},
    "resolution": [1920, 1080],
    "render_mode": "rtx_realtime",         # rtx_realtime | path_traced
    "path_traced_spp": 64,
    "warmup_subframes": 4,
    "check": False,
    "output_prefix": "replay_",
    "cameras": {},
}

HORIZONTAL_APERTURE_MM = 20.955
LOG_EVERY = 50


# ---------------------------------------------------------------------------
# pure helpers (no Isaac imports)
# ---------------------------------------------------------------------------

def load_config(path: Optional[str | Path]) -> dict:
    import yaml
    cfg = json.loads(json.dumps(DEFAULTS))
    if path:
        user = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict) and k != "part_map" and k != "cameras":
                cfg[k].update(v)
            else:
                cfg[k] = v
    if not cfg.get("stage"):
        raise ValueError("config: 'stage' is required")
    return cfg


def load_state(path: str | Path) -> dict:
    d = np.load(path, allow_pickle=False)
    st = {k: d[k] for k in d.files}
    st["meta"] = json.loads(str(st["meta"])) if "meta" in st else {}
    if st["meta"].get("quaternion_order", "wxyz") != "wxyz":
        raise ValueError("npz quaternions are not wxyz")
    return st


def _norm(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def look_at_rows(eye, target, up) -> list[list[float]]:
    """Camera-to-world 4x4 in USD row-vector form. USD cameras look down -Z with +Y up."""
    eye, target, up = (np.asarray(x, float) for x in (eye, target, up))
    f = _norm(target - eye)
    r = np.cross(f, up)
    if np.linalg.norm(r) < 1e-6:
        raise ValueError(f"camera up vector {up.tolist()} is parallel to the view direction; set 'up' in the config")
    r = _norm(r)
    u = np.cross(r, f)
    return [[*r, 0.0], [*u, 0.0], [*(-f), 0.0], [*eye, 1.0]]


def _catmull(p0, p1, p2, p3, s):
    s2, s3 = s * s, s * s * s
    return 0.5 * ((2 * p1) + (-p0 + p2) * s + (2 * p0 - 5 * p1 + 4 * p2 - p3) * s2 + (-p0 + 3 * p1 - 3 * p2 + p3) * s3)


def camera_at(cam: dict, t: float) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """(position, look_at, focal_length_mm, up) at replay time t (seconds, t_rel)."""
    up = np.asarray(cam.get("up", [0.0, 0.0, 1.0]), float)
    if "keyframes" not in cam:
        return (np.asarray(cam["position"], float), np.asarray(cam["look_at"], float),
                float(cam.get("focal_length", 24.0)), up)
    ks = sorted(cam["keyframes"], key=lambda k: float(k["t"]))
    ts = [float(k["t"]) for k in ks]
    P = [np.asarray(k["position"], float) for k in ks]
    L = [np.asarray(k["look_at"], float) for k in ks]
    F = [float(k.get("focal_length", cam.get("focal_length", 24.0))) for k in ks]
    if t <= ts[0] or len(ks) == 1:
        return P[0], L[0], F[0], up
    if t >= ts[-1]:
        return P[-1], L[-1], F[-1], up
    i = max(j for j in range(len(ts) - 1) if ts[j] <= t)
    s = (t - ts[i]) / (ts[i + 1] - ts[i])
    idx = [max(i - 1, 0), i, i + 1, min(i + 2, len(ks) - 1)]
    pos = _catmull(*(P[j] for j in idx), s)
    look = _catmull(*(L[j] for j in idx), s)
    focal = F[i] + (F[i + 1] - F[i]) * (s * s * (3 - 2 * s))
    return pos, look, focal, up


def write_png(path: Path, rgba: np.ndarray) -> None:
    """PNG writer: Pillow if importable, else a stdlib zlib encoder."""
    rgb = np.ascontiguousarray(rgba[..., :3].astype(np.uint8))
    try:
        from PIL import Image
        Image.fromarray(rgb, "RGB").save(path)
        return
    except ImportError:
        pass
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) \
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    path.write_bytes(png)


def parse_urdf_mimic(urdf_path: str) -> dict[str, tuple[str, float, float]]:
    """joint -> (reference joint, multiplier, offset) from URDF <mimic> tags (URDF convention)."""
    import xml.etree.ElementTree as ET
    out = {}
    for j in ET.parse(urdf_path).getroot().iter("joint"):
        m = j.find("mimic")
        if m is not None:
            out[j.get("name")] = (m.get("joint"), float(m.get("multiplier", 1.0)), float(m.get("offset", 0.0)))
    return out


# ---------------------------------------------------------------------------
# Isaac layer
# ---------------------------------------------------------------------------

class Replay:
    def __init__(self, state: dict, cfg: dict, check: bool, overwrite: bool) -> None:
        self.st, self.cfg = state, cfg
        self.check = bool(check or cfg.get("check"))
        self.overwrite = overwrite
        self.session_dir = Path(state["meta"].get("session_dir") or ".")
        n = len(state["t_rel"])
        self.ticks = sorted({0, n // 2, n - 1}) if self.check else list(range(n))
        self.warnings: list[str] = []
        self.cams: dict[str, dict] = {}
        self.parts: list[tuple[str, int, Any]] = []      # (prim_path, frame index, prim wrapper)
        self.frame_index = {str(f): i for i, f in enumerate(state["frame_names"])}
        self.joint_index = {str(j): i for i, j in enumerate(state["joint_names"])}

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)
        print(f"[replay] warning: {msg}")

    # ---- stage ---------------------------------------------------------
    def lock_and_edit_session_layer(self, stage) -> None:
        from pxr import Usd
        for layer in stage.GetUsedLayers():
            try:
                layer.SetPermissionToSave(False)
            except Exception:  # noqa: BLE001  anonymous layers refuse; they are never saved anyway
                pass
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        print(f"[replay] stage {stage.GetRootLayer().identifier}: layers locked against saving, edits go to the session layer")

    def make_physics_free(self, stage) -> None:
        from pxr import Usd, UsdPhysics
        for path in self.cfg.get("deactivate_prims") or []:
            prim = stage.GetPrimAtPath(path)
            if prim:
                prim.SetActive(False)
            else:
                self.warn(f"deactivate_prims: {path} not found")
        n_coll = n_kin = n_dis = 0
        part_roots = {str(p) for p in (self.cfg.get("part_map") or {}).values()}
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.IsA(UsdPhysics.Scene):
                UsdPhysics.Scene(prim).GetGravityMagnitudeAttr().Set(0.0)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                n_coll += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                path = str(prim.GetPath())
                in_part = any(path == r or path.startswith(r + "/") for r in part_roots)
                if in_part and self.cfg.get("part_physics") == "disable":
                    UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
                    n_dis += 1
                else:
                    UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)
                    UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set((0.0, 0.0, 0.0))
                    UsdPhysics.RigidBodyAPI(prim).GetAngularVelocityAttr().Set((0.0, 0.0, 0.0))
                    n_kin += 1
        print(f"[replay] gravity 0, {n_coll} colliders disabled, {n_kin} rigid bodies kinematic, {n_dis} disabled")

    def make_cameras(self, stage) -> None:
        from pxr import Gf, UsdGeom
        w, h = (int(x) for x in self.cfg["resolution"])
        UsdGeom.Xform.Define(stage, "/Replay")
        for name, cam in (self.cfg.get("cameras") or {}).items():
            path = cam.get("prim") or f"/Replay/Cameras/{name}"
            if cam.get("prim"):
                prim = stage.GetPrimAtPath(path)
                if not prim:
                    raise ValueError(f"camera {name}: prim {path} not found")
                camera = UsdGeom.Camera(prim)
                moving = "position" in cam or "keyframes" in cam
            else:
                camera = UsdGeom.Camera.Define(stage, path)
                moving = True
            camera.GetHorizontalApertureAttr().Set(float(cam.get("horizontal_aperture", HORIZONTAL_APERTURE_MM)))
            camera.GetVerticalApertureAttr().Set(float(cam.get("horizontal_aperture", HORIZONTAL_APERTURE_MM)) * h / w)
            camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
            xf = UsdGeom.Xformable(camera.GetPrim())
            op = None
            if moving:
                xf.ClearXformOpOrder()
                op = xf.AddTransformOp()
            out_dir = self.session_dir / "frames" / f"{self.cfg.get('output_prefix', 'replay_')}{name}{'_check' if self.check else ''}"
            if (out_dir / "frames.jsonl").exists() and not self.overwrite:
                raise FileExistsError(f"{out_dir}/frames.jsonl exists; pass overwrite=True / --overwrite to replace it")
            out_dir.mkdir(parents=True, exist_ok=True)
            if self.overwrite:
                for old in out_dir.glob("*.png"):
                    old.unlink()
                (out_dir / "frames.jsonl").unlink(missing_ok=True)
            self.cams[name] = {"cfg": cam, "camera": camera, "op": op, "dir": out_dir, "moving": moving,
                               "path": path, "index": 0}

    def set_cameras(self, t: float) -> None:
        from pxr import Gf
        for c in self.cams.values():
            if not c["moving"]:
                continue
            pos, look, focal, up = camera_at(c["cfg"], t)
            rows = look_at_rows(pos, look, up)
            c["op"].Set(Gf.Matrix4d(*[float(v) for row in rows for v in row]))
            c["camera"].GetFocalLengthAttr().Set(float(focal))

    def bind_parts(self, stage) -> None:
        from isaacsim.core.prims import SingleXFormPrim
        for frame, prim_path in (self.cfg.get("part_map") or {}).items():
            if not stage.GetPrimAtPath(prim_path):
                self.warn(f"part_map: prim {prim_path} (tf {frame}) not in stage")
                continue
            if frame not in self.frame_index:
                self.warn(f"part_map: tf frame {frame} not in the npz; {prim_path} will not move")
                continue
            self.parts.append((prim_path, self.frame_index[frame], SingleXFormPrim(prim_path)))

    # ---- robot -----------------------------------------------------------
    def bind_robot(self, stage) -> None:
        from isaacsim.core.prims import SingleArticulation
        self.art = SingleArticulation(prim_path=self.cfg["articulation_root"], name="replay_robot")
        self.art.initialize()
        dofs = list(self.art.dof_names)
        self.dof_names = dofs
        self.q0 = np.array(self.art.get_joint_positions(), float)
        self.dof_src: list[tuple[str, Any]] = []
        mimic = self._mimic_relations(stage, dofs)
        unset = []
        for name in dofs:
            if name in self.joint_index:
                self.dof_src.append(("bag", self.joint_index[name]))
            elif name in mimic and mimic[name][0] in self.joint_index:
                ref, g, off = mimic[name]
                self.dof_src.append(("mimic", (self.joint_index[ref], g, off)))
            else:
                self.dof_src.append(("hold", None))
                unset.append(name)
        print(f"[replay] articulation {self.cfg['articulation_root']}: {len(dofs)} DOFs, "
              f"{sum(1 for s, _ in self.dof_src if s == 'bag')} from the bag, "
              f"{sum(1 for s, _ in self.dof_src if s == 'mimic')} from mimic relations")
        for name in unset:
            self.warn(f"DOF {name} is neither in /joint_states nor a resolvable mimic joint; held at its initial value")
        self._mimic_selfcheck(mimic)

    def _mimic_relations(self, stage, dofs: list[str]) -> dict[str, tuple[str, float, float]]:
        """joint -> (reference joint, gain, offset_rad), normalised to q = gain * q_ref + offset."""
        m = self.cfg.get("mimic") or {}
        if m.get("source") == "urdf":
            if not m.get("urdf_path"):
                self.warn("mimic.source is urdf but mimic.urdf_path is empty")
                return {}
            return parse_urdf_mimic(m["urdf_path"])
        from pxr import Usd
        root = stage.GetPrimAtPath(self.cfg["robot_prim"])
        out = {}
        for prim in Usd.PrimRange(root):
            name = prim.GetName()
            if name not in dofs:
                continue
            for axis in ("rotX", "rotY", "rotZ", "transX", "transY", "transZ"):
                g_attr = prim.GetAttribute(f"physxMimicJoint:{axis}:gearing")
                rel = prim.GetRelationship(f"physxMimicJoint:{axis}:referenceJoint")
                if not g_attr or not rel or not rel.GetTargets():
                    continue
                g = float(g_attr.Get() or 0.0)
                off_attr = prim.GetAttribute(f"physxMimicJoint:{axis}:offset")
                off = float(off_attr.Get() or 0.0) if off_attr else 0.0
                if axis.startswith("rot") and m.get("offset_units", "deg") == "deg":
                    off = math.radians(off)
                ref = rel.GetTargets()[0].name
                if m.get("convention", "physx") == "physx":
                    # PhysX articulation mimic constraint: q + gearing * q_ref + offset = 0
                    out[name] = (ref, -g, -off)
                else:
                    out[name] = (ref, g, off)
        return out

    def _mimic_selfcheck(self, mimic: dict) -> None:
        """When the bag ALSO has a mimic joint, report how well the relation predicts it."""
        jp = self.st["joint_pos"]
        for name, (ref, g, off) in mimic.items():
            if name in self.joint_index and ref in self.joint_index:
                pred = g * jp[:, self.joint_index[ref]] + off
                res = float(np.max(np.abs(pred - jp[:, self.joint_index[name]])))
                alt = float(np.max(np.abs(-g * jp[:, self.joint_index[ref]] - off - jp[:, self.joint_index[name]])))
                print(f"[replay] mimic self-check {name}: max |relation - bag| = {res:.4g} rad "
                      f"(opposite sign convention: {alt:.4g} rad); the bag value is used")

    def apply_tick(self, k: int) -> None:
        from isaacsim.core.utils.types import ArticulationAction
        jp = self.st["joint_pos"][k]
        q = self.q0.copy()
        for i, (src, ref) in enumerate(self.dof_src):
            if src == "bag":
                q[i] = jp[ref]
            elif src == "mimic":
                j, g, off = ref
                q[i] = g * jp[j] + off
        self.art.set_joint_positions(q)
        self.art.set_joint_velocities(np.zeros_like(q))
        self.art.apply_action(ArticulationAction(joint_positions=q))
        pos, quat = self.st["frame_pos"][k], self.st["frame_quat"][k]
        for _, fi, xp in self.parts:
            xp.set_world_pose(position=pos[fi], orientation=quat[fi])
        self.set_cameras(float(self.st["t_rel"][k]))

    # ---- rendering -------------------------------------------------------
    def setup_render(self) -> None:
        import omni.replicator.core as rep
        try:
            rep.orchestrator.set_capture_on_play(False)
        except Exception:  # noqa: BLE001
            pass
        if self.cfg.get("render_mode") == "path_traced":
            rep.settings.set_render_pathtraced(samples_per_pixel=int(self.cfg.get("path_traced_spp", 64)))
        else:
            rep.settings.set_render_rtx_realtime()
        res = tuple(int(x) for x in self.cfg["resolution"])
        for c in self.cams.values():
            rp = rep.create.render_product(c["path"], res)
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach([rp])
            c["rp"], c["annot"] = rp, annot

    def write_tick(self, k: int) -> None:
        for name, c in self.cams.items():
            data = c["annot"].get_data()
            arr = np.asarray(data["data"] if isinstance(data, dict) else data)
            if arr.size == 0:
                self.warn(f"camera {name}: empty image at tick {k}")
                continue
            fname = f"{c['index']:06d}.png"
            write_png(c["dir"] / fname, arr)
            rec = {"index": c["index"], "file": fname, "wall_time_iso": str(self.st["wall_time_iso"][k]),
                   "log_time_ns": int(self.st["log_time_ns"][k]), "t_rel": float(self.st["t_rel"][k]),
                   "camera": f"{self.cfg.get('output_prefix', 'replay_')}{name}", "tick": int(k)}
            with open(c["dir"] / "frames.jsonl", "a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec) + "\n")
            c["index"] += 1

    def summary(self, t_start: float) -> None:
        n = sum(c["index"] for c in self.cams.values())
        unmapped_frames = [str(f) for f in self.st["frame_names"] if str(f) not in (self.cfg.get("part_map") or {})]
        bag_joints_unused = [j for j in self.joint_index if j not in getattr(self, "dof_names", [])]
        print(f"[replay] done: {n} frames over {len(self.cams)} camera(s) in {time.time() - t_start:.1f} s"
              f"{' (check mode: first, middle, last)' if self.check else ''}")
        for name, c in self.cams.items():
            print(f"[replay]   {name}: {c['index']} frames -> {c['dir']}")
        print(f"[replay] TF frames in the npz not mapped to a part prim: {len(unmapped_frames)} (robot links are driven by joints)")
        if bag_joints_unused:
            print(f"[replay] bag joints with no articulation DOF: {bag_joints_unused}")
        for w in self.warnings:
            print(f"[replay] warning: {w}")


def _rep_step_kwargs(warmup: int) -> dict:
    return {"rt_subframes": max(1, int(warmup)), "delta_time": 0.0, "pause_timeline": False}


# ---------------------------------------------------------------------------
# drivers
# ---------------------------------------------------------------------------

async def run_async(state_path: str, config_path: str, check: bool = False, overwrite: bool = False) -> None:
    """Script Editor entry point: uses the running Isaac app. Opens the configured stage."""
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.core.api import World

    cfg = load_config(config_path)
    st = load_state(state_path)
    ctx = omni.usd.get_context()
    try:
        if ctx.has_pending_edit():
            raise RuntimeError("the open stage has unsaved edits; save or discard them before replaying")
    except AttributeError:
        pass
    ok, err = await ctx.open_stage_async(str(cfg["stage"]))
    if not ok:
        raise RuntimeError(f"could not open {cfg['stage']}: {err}")
    stage = ctx.get_stage()
    r = Replay(st, cfg, check, overwrite)
    r.lock_and_edit_session_layer(stage)
    r.make_physics_free(stage)
    r.make_cameras(stage)
    if World.instance() is not None:
        World.instance().clear_instance()
    world = World(stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()
    await world.reset_async()
    r.bind_robot(stage)
    r.bind_parts(stage)
    r.setup_render()
    t0 = time.time()
    for n, k in enumerate(r.ticks):
        r.apply_tick(k)
        world.step(render=False)
        try:
            await rep.orchestrator.step_async(**_rep_step_kwargs(cfg["warmup_subframes"]))
        except TypeError:
            await rep.orchestrator.step_async(rt_subframes=max(1, int(cfg["warmup_subframes"])))
        r.write_tick(k)
        if n % LOG_EVERY == 0:
            print(f"[replay] {n}/{len(r.ticks)} t_rel {float(st['t_rel'][k]):.1f} s")
    await rep.orchestrator.wait_until_complete_async()
    world.pause()
    r.summary(t0)


def run_standalone(argv: Optional[list[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="render_replay")
    ap.add_argument("--state", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--headless", action="store_true", default=True)
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    w, h = (int(x) for x in cfg["resolution"])

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": a.headless, "width": w, "height": h})
    try:
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import is_stage_loading, open_stage

        st = load_state(a.state)
        if not open_stage(str(cfg["stage"])):
            raise RuntimeError(f"could not open {cfg['stage']}")
        while is_stage_loading():
            app.update()
        stage = omni.usd.get_context().get_stage()
        r = Replay(st, cfg, a.check, a.overwrite)
        r.lock_and_edit_session_layer(stage)
        r.make_physics_free(stage)
        r.make_cameras(stage)
        world = World(stage_units_in_meters=1.0)
        world.reset()
        r.bind_robot(stage)
        r.bind_parts(stage)
        r.setup_render()
        t0 = time.time()
        for n, k in enumerate(r.ticks):
            r.apply_tick(k)
            world.step(render=False)
            try:
                rep.orchestrator.step(**_rep_step_kwargs(cfg["warmup_subframes"]))
            except TypeError:
                rep.orchestrator.step(rt_subframes=max(1, int(cfg["warmup_subframes"])))
            r.write_tick(k)
            if n % LOG_EVERY == 0:
                print(f"[replay] {n}/{len(r.ticks)} t_rel {float(st['t_rel'][k]):.1f} s", flush=True)
        rep.orchestrator.wait_until_complete()
        r.summary(t0)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(run_standalone())
