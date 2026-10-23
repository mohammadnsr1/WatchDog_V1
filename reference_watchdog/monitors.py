"""Reference scene-observation monitors for the WatchDog authored-monitor lane.

These are the monitors a competent watchdog SHOULD write with the data
streams the cell already publishes. They exist to prove that the scene
observation duty (unlisted object appears, external body visits and leaves,
object moved when the robot did not move it) is achievable with
`ctx.depth()`, `ctx.camera()`, `ctx.deproject()`, `ctx.frame_pose("tcp")`,
`ctx.object_pose()`, `ctx.joint_window()` and `ctx.held_object()` alone --
nothing the agents do not also have.

Both sources are written against the authored-monitor sandbox exactly as
`arm_custom_monitor` enforces it: no imports (numpy is PROVIDED as `np` on
the array lane), no while loops, no underscore names, top level holds only
`def`s and literal constants, under 8000 characters, 150 ms per call.

SCENE_WATCH (array lane, 1 Hz, purpose="observation")
    Depth-based. Height above the bench plane is computed per pixel from the
    camera intrinsics/extrinsics and the known bench top. The robot is masked
    as the high region CONNECTED to the TCP's projected pixel (plus a disc
    around the TCP for the gripper and a carried part); a visitor's forearm
    or body is high too but not connected to the arm, so it is not masked.
    The reference frame is captured immediately with the arm region unknown
    (NaN) and filled in as the arm reveals the bench. Three fire kinds:
      new_static  -- unexplained rise over the reference, static for PERSIST
                     ticks (a foreign object placed on the bench)
      removed     -- the reference had something there and now it is gone
      moving      -- unexplained pixels in motion for MOVE_PERSIST ticks
                     (a visitor entering or leaving)
    A change whose world XY lies within EXPLAIN_R of a registered object is
    EXPLAINED (the executor placed, picked, dropped or toppled a known part)
    and is folded into the reference without firing. The monitor disarms on
    fire by contract; the scripted client re-arms it.

UNDISTURBED_WATCH (scalar lane, purpose="observation")
    TF-based. Every registered object's position is tracked; while the
    gripper holds it or the TCP is within NEAR_TCP of it, the reference
    follows it (the robot is allowed to move it). A displacement beyond
    MOVE_TOL while the TCP is far and the object is not held is an external
    displacement. Fires once (object_id, delta) and is re-armed by the client.

Each fire's `detail` is a flat JSON object encoded by hand (the sandbox has
no json module) so the client can parse it with json.loads.
"""

from __future__ import annotations

from typing import Any

BENCH_TOP_Z = 0.01  # inject_intruder.BENCH_TOP_Z: bench surface in world z

SCENE_WATCH_SOURCE = r'''
STEP = 4
BENCH_Z = 0.01
H_MIN = 0.015
ARM_H = 0.10
R_TCP_M = 0.16
PERSIST = 3
MOVE_PERSIST = 2
MIN_CELLS = 6
MIN_MOVE_CELLS = 30
EXPLAIN_R = 0.10
GROW_ITERS = 80

def rot_from_quat(q):
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])

def grow(seed, within):
    g = seed.copy()
    g[1:, :] |= seed[:-1, :]
    g[:-1, :] |= seed[1:, :]
    g[:, 1:] |= seed[:, :-1]
    g[:, :-1] |= seed[:, 1:]
    return g & within

def connected(seed, within):
    comp = seed & within
    for i in range(GROW_ITERS):
        nxt = grow(comp, within)
        if nxt.sum() == comp.sum():
            return comp
        comp = nxt
    return comp

def setup(ctx, cam, shape):
    k = cam["K"]
    fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
    t = np.array(cam["world_T_camera"]["translation"], dtype=float)
    rot = rot_from_quat(cam["world_T_camera"]["rotation_xyzw"])
    vs, us = np.mgrid[0:shape[0], 0:shape[1]]
    u = us * STEP + (STEP - 1) / 2.0
    v = vs * STEP + (STEP - 1) / 2.0
    dirs = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones(u.shape)], axis=-1) @ rot.T
    dz = dirs[..., 2]
    dz = np.where(np.abs(dz) < 1e-6, -1e-6, dz)
    ctx.state["bench_depth"] = ((BENCH_Z - t[2]) / dz).astype(np.float32)
    ctx.state["dz"] = (-dz).astype(np.float32)
    ctx.state["fx"], ctx.state["fy"], ctx.state["cx"], ctx.state["cy"] = fx, fy, cx, cy
    ctx.state["rot_t"] = rot.T.copy()
    ctx.state["t"] = t
    ctx.state["px_per_m"] = fx / max(float(abs(t[2] - BENCH_Z)), 0.5)

def world_to_cell(ctx, p):
    st = ctx.state
    rel = np.array([p["x"], p["y"], p["z"]], dtype=float) - st["t"]
    c = st["rot_t"] @ rel
    if c[2] <= 0.05:
        return None
    u = st["fx"] * c[0] / c[2] + st["cx"]
    v = st["fy"] * c[1] / c[2] + st["cy"]
    return (v / STEP, u / STEP)

def robot_mask(ctx, height, valid):
    high = valid & (height > ARM_H)
    tcp = ctx.frame_pose("tcp")
    disc = np.zeros(height.shape, dtype=bool)
    if tcp is not None:
        cell = world_to_cell(ctx, tcp["pose"]["position"])
        if cell is not None:
            vs, us = np.mgrid[0:height.shape[0], 0:height.shape[1]]
            r = ctx.state["px_per_m"] * R_TCP_M / STEP
            disc = ((vs - cell[0]) ** 2 + (us - cell[1]) ** 2) <= r * r
            arm = connected(disc & high, high) | disc
            arm[0:2, :] |= high[0:2, :]
            return grow(grow(grow(arm, np.ones(arm.shape, dtype=bool)),
                             np.ones(arm.shape, dtype=bool)),
                        np.ones(arm.shape, dtype=bool))
    return grow(grow(high, np.ones(high.shape, dtype=bool)),
                np.ones(high.shape, dtype=bool))

def track_objects(ctx):
    seen = ctx.state.setdefault("seen", {})
    for oid in ctx.object_ids():
        p = ctx.object_pose(oid)
        if p is None:
            continue
        pos = p["pose"]["position"]
        hist = seen.setdefault(oid, [])
        if not hist or math.hypot(hist[-1][0] - pos["x"], hist[-1][1] - pos["y"]) > 0.02:
            hist.append([pos["x"], pos["y"]])

def explained(ctx, world, kind):
    seen = ctx.state.get("seen", {})
    for oid in ctx.object_ids():
        spots = seen.get(oid, [])
        if kind != "removed":
            spots = spots[-1:]
        for xy in spots:
            if math.hypot(xy[0] - world["x"], xy[1] - world["y"]) < EXPLAIN_R:
                return oid
    return None

def blob_report(ctx, kind, blob, d, ref, since_ns):
    st = ctx.state
    vs, us = np.nonzero(blob)
    src = d if kind != "removed" else ref
    z = float(np.nanmedian(src[blob]))
    u = float(us.mean()) * STEP + (STEP - 1) / 2.0
    v = float(vs.mean()) * STEP + (STEP - 1) / 2.0
    w = ctx.deproject(u, v, z)
    h = float(np.nanmedian(((st["bench_depth"] - src) * st["dz"])[blob]))
    area_px = int(blob.sum()) * STEP * STEP
    side_m = math.sqrt(area_px) / st["px_per_m"]
    detail = ('{"kind":"' + kind + '","u":' + str(round(u, 1)) + ',"v":' + str(round(v, 1))
              + ',"x":' + str(round(w["x"], 3)) + ',"y":' + str(round(w["y"], 3))
              + ',"z":' + str(round(w["z"], 3)) + ',"height_m":' + str(round(h, 3))
              + ',"side_m":' + str(round(side_m, 3)) + ',"area_px":' + str(area_px)
              + ',"since_wall_ns":' + str(int(since_ns)) + '}')
    return w, {"value": float(area_px), "detail": detail}

def check(ctx):
    d = ctx.depth()
    cam = ctx.camera()
    if d is None or cam is None:
        return None
    d = d[::STEP, ::STEP]
    st = ctx.state
    now = ctx.now_wall_ns()
    if "bench_depth" not in st or st["bench_depth"].shape != d.shape:
        setup(ctx, cam, d.shape)
        st["ref"] = np.full(d.shape, np.nan, dtype=np.float32)
        st["age"] = np.zeros(d.shape, dtype=np.int16)
        st["stable"] = np.zeros(d.shape, dtype=np.int16)
        st["since"] = np.zeros(d.shape, dtype=np.int64)
        st["move_ticks"] = 0
        st["move_since"] = 0
        st["prev"] = d.copy()
    track_objects(ctx)
    valid = np.isfinite(d)
    height = (st["bench_depth"] - d) * st["dz"]
    mask = robot_mask(ctx, np.where(valid, height, 0.0), valid)
    ref = st["ref"]
    known = np.isfinite(ref)
    rise = np.where(known, ref - d, np.where(valid, height, 0.0))
    if "init" not in st:
        st["init"] = True
        ref[valid & ~mask] = d[valid & ~mask]
        st["prev"] = d.copy()
        return None
    prev = st["prev"]
    still = valid & np.isfinite(prev) & (np.abs(d - prev) < H_MIN)
    st["stable"] = np.where(still, st["stable"] + 1, 0)
    st["prev"] = d.copy()
    changed = valid & ~mask & (np.abs(rise) > H_MIN)
    fill = valid & ~mask & ~known & (st["stable"] >= PERSIST) & ~changed
    ref[fill] = d[fill]
    moving = valid & ~mask & ~still & (np.abs(rise) > H_MIN)
    st["age"] = np.where(changed & ~mask, st["age"] + 1, 0)
    st["since"] = np.where(st["age"] == 1, now, st["since"])
    hit = st["age"] >= PERSIST
    if hit.sum() >= MIN_CELLS:
        flat = np.argmax(np.where(hit, st["age"], 0))
        seed = np.zeros(hit.shape, dtype=bool)
        seed[flat // hit.shape[1], flat % hit.shape[1]] = True
        blob = connected(seed, hit)
        if blob.sum() >= MIN_CELLS:
            kind = "new_static" if float(np.nanmedian(rise[blob])) > 0 else "removed"
            w, report = blob_report(ctx, kind, blob, d, ref, st["since"][blob].min())
            ref[blob] = d[blob]
            st["age"][blob] = 0
            if explained(ctx, w, kind) is None:
                return report
        else:
            st["age"][blob] = 0
    if moving.sum() >= MIN_MOVE_CELLS:
        st["move_ticks"] += 1
        if st["move_ticks"] == 1:
            st["move_since"] = now
        if st["move_ticks"] >= MOVE_PERSIST:
            w, report = blob_report(ctx, "moving", moving, d, ref, st["move_since"])
            st["move_ticks"] = 0
            if explained(ctx, w, "moving") is None:
                return report
    else:
        st["move_ticks"] = 0
    return None
'''

UNDISTURBED_WATCH_SOURCE = r'''
MOVE_TOL = 0.01
NEAR_TCP = 0.12

def check(ctx):
    st = ctx.state
    refs = st.setdefault("refs", {})
    tcp = ctx.frame_pose("tcp")
    tcp_pos = tcp["pose"]["position"] if tcp is not None else None
    held = ctx.held_object()
    for oid in ctx.object_ids():
        p = ctx.object_pose(oid)
        if p is None:
            continue
        pos = p["pose"]["position"]
        robot_may_move_it = (held == oid) or (
            tcp_pos is not None
            and math.hypot(tcp_pos["x"] - pos["x"], tcp_pos["y"] - pos["y"]) < NEAR_TCP
        )
        if oid not in refs or robot_may_move_it:
            refs[oid] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
            continue
        delta = ctx.dist(refs[oid], pos)
        if delta > ctx.value("move_tol_m"):
            ref = refs[oid]
            refs[oid] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
            detail = ('{"kind":"object_moved","object_id":"' + oid + '"'
                      + ',"x":' + str(round(pos["x"], 3)) + ',"y":' + str(round(pos["y"], 3))
                      + ',"z":' + str(round(pos["z"], 3))
                      + ',"from_x":' + str(round(ref["x"], 3)) + ',"from_y":' + str(round(ref["y"], 3))
                      + ',"delta_m":' + str(round(delta, 4))
                      + ',"since_wall_ns":' + str(int(p.get("wall_ns") or ctx.now_wall_ns())) + '}')
            return {"value": delta, "detail": detail}
    return None
'''


def scene_watch_spec() -> dict[str, Any]:
    """Keyword arguments for arm_custom_monitor, SCENE_WATCH."""
    return {
        "monitor_id": "ref_scene_watch",
        "code": SCENE_WATCH_SOURCE,
        "description": (
            "Reference scene watch: depth height-above-bench vs a reference "
            "frame, robot masked by TCP-connected high region, changes "
            "unexplained by any registered object fire as new_static / "
            "removed / moving."
        ),
        "params": {
            "min_rise_m": {"value": 0.015, "source": None},
        },
        "clause": None,
        "object_id": None,
        "threshold_param": "min_rise_m",
        "rate_hz": 1.0,
        "purpose": "observation",
        "threshold_rationale": (
            "0.015 m is above the simulated depth quantisation and below the "
            "smallest FMB part height (about 0.02 m) and the 0.05 m intruder "
            "cube, so a resting foreign object is a clear rise while bench "
            "texture is not. Chosen by the operator for the reference run."
        ),
        "halt_on_fire": False,
    }


def undisturbed_watch_spec() -> dict[str, Any]:
    """Keyword arguments for arm_custom_monitor, UNDISTURBED_WATCH."""
    return {
        "monitor_id": "ref_undisturbed_watch",
        "code": UNDISTURBED_WATCH_SOURCE,
        "description": (
            "Reference undisturbed-object watch: a registered object that "
            "moves while not held and with the TCP far from it was moved by "
            "something other than the robot."
        ),
        "params": {
            "move_tol_m": {"value": 0.01, "source": None},
        },
        "clause": None,
        "object_id": None,
        "threshold_param": "move_tol_m",
        "rate_hz": None,
        "purpose": "observation",
        "threshold_rationale": (
            "0.01 m is well above TF jitter on a resting part and half the "
            "0.02 m disturbance tolerance the contracts use, so an external "
            "nudge is caught before it becomes a conformance question. "
            "Chosen by the operator for the reference run."
        ),
        "halt_on_fire": False,
    }


ALL_SPECS = (scene_watch_spec, undisturbed_watch_spec)
