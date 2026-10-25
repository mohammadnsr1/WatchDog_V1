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
    camera intrinsics/extrinsics and the known bench top. Where no reference
    exists yet, only a surface ABOVE the plane can be a candidate: below it
    is the floor past the bench edge or a hole, never a removed object (live
    2026-10-23: the floor corner uncovered by the arm fired as "removed").
    A static blob uncovered from under the robot's mask that lies within
    REVEAL_GAP cells of any high structure (robot, base, gantry) is the
    robot's own and is folded in silently (live 2026-10-23: the base flange
    uncovered by the move to home fired as "new_static"); an uncovered
    object clear of the robot is still reported. The robot is masked
    GEOMETRICALLY: every UR link origin (base_link .. wrist_3_link, tcp) is
    read from TF through ctx.transform and projected into the image, and the
    mask is the union of tubes along consecutive links. The radius is per
    LINK POINT (RAD: 0.16 base, 0.20 shoulder, 0.22 elbow housing, 0.15
    wrists, 0.16 tcp disc for the gripper and a carried part) and is
    interpolated along each segment and scaled by the depth at the closest
    point, never by the segment's nearest depth (live 2026-10-24: the
    near-vertical upper arm, sized at the elbow's depth, hid the cube spot
    30 cells from the shoulder for the whole run). A
    visitor's forearm or body is masked only where it overlaps the arm's
    actual silhouette. (The first live run, 2026-10-24, masked "the high
    region connected to the TCP"; with the gripper down at bench level the
    connection broke, the whole arm fired as a 30-60 cm object and was
    folded into the reference, which then produced garbage all run.) Motion
    is evaluated MOVE_GAP cells further from the arm than static change,
    since the arm's silhouette edge is where motion artefacts live.
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
BZ = 0.01
HMIN = 0.015
ARMH = 0.10
PER = 3
MPER = 2
MINC = 6
MCELLS = 30
EXR = 0.10
ITERS = 80
RGAP = 6
RAD = [0.16, 0.20, 0.20, 0.22, 0.15, 0.15, 0.14, 0.16]
TALL_H = 0.25
TGAP = 10
ZY = -0.60
MOVE_GAP = 3
LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
         "wrist_1_link", "wrist_2_link", "wrist_3_link", "tcp"]

def quat_rot(q):
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

def dilate(m, n):
    ones = np.ones(m.shape, dtype=bool)
    for i in range(n):
        m = grow(m, ones)
    return m

def connected(seed, within):
    comp = seed & within
    for i in range(ITERS):
        nxt = grow(comp, within)
        if nxt.sum() == comp.sum():
            return comp
        comp = nxt
    return comp

def setup(ctx, cam, shape):
    k = cam["K"]
    fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
    t = np.array(cam["world_T_camera"]["translation"], dtype=float)
    rot = quat_rot(cam["world_T_camera"]["rotation_xyzw"])
    vs, us = np.mgrid[0:shape[0], 0:shape[1]]
    u = us * STEP + (STEP - 1) / 2.0
    v = vs * STEP + (STEP - 1) / 2.0
    dirs = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones(u.shape)], axis=-1) @ rot.T
    dz = dirs[..., 2]
    dz = np.where(np.abs(dz) < 1e-6, -1e-6, dz)
    st = ctx.state
    st["bdepth"] = ((BZ - t[2]) / dz).astype(np.float32)
    st["dz"] = (-dz).astype(np.float32)
    st["fx"], st["fy"], st["cx"], st["cy"] = fx, fy, cx, cy
    st["rot_t"] = rot.T.copy()
    st["t"] = t
    st["ppm"] = fx / max(float(abs(t[2] - BZ)), 0.5)
    st["grid"] = (vs.astype(float), us.astype(float))

def cpt(ctx, p):
    st = ctx.state
    rel = np.array([p["x"], p["y"], p["z"]], dtype=float) - st["t"]
    c = st["rot_t"] @ rel
    if c[2] <= 0.05:
        return None
    return ((st["fy"] * c[1] / c[2] + st["cy"]) / STEP,
            (st["fx"] * c[0] / c[2] + st["cx"]) / STEP, c[2])

def tube(gv, gu, a, b, ra, rb, k):
    dv, du = b[0] - a[0], b[1] - a[1]
    l2 = dv * dv + du * du
    s = np.zeros(gv.shape) if l2 < 1e-9 else np.clip(((gv - a[0]) * dv + (gu - a[1]) * du) / l2, 0.0, 1.0)
    r = k * (ra + s * (rb - ra)) / (a[2] + s * (b[2] - a[2]))
    return (gv - (a[0] + s * dv)) ** 2 + (gu - (a[1] + s * du)) ** 2 <= r * r

def rmask(ctx, hgt, valid):
    st = ctx.state
    gv, gu = st["grid"]
    pts = []
    for name in LINKS:
        try:
            pts.append(cpt(ctx, ctx.transform("world", name)["translation"]))
        except:
            pts.append(None)
    m = np.zeros(hgt.shape, dtype=bool)
    k = st["fx"] / STEP
    for i in range(len(RAD) - 1):
        a, b = pts[i], pts[i + 1]
        if a is not None and b is not None:
            m |= tube(gv, gu, a, b, RAD[i], RAD[i + 1], k)
    if pts[-1] is not None:
        m |= tube(gv, gu, pts[-1], pts[-1], RAD[-1], RAD[-1], k)
    if not m.any():
        return dilate(valid & (hgt > ARMH), 2)
    return dilate(m, 3) | (valid & (hgt > TALL_H) & dilate(m, TGAP))

def track(ctx):
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
            if math.hypot(xy[0] - world["x"], xy[1] - world["y"]) < EXR:
                return oid
    return None

def rep(ctx, kind, blob, d, ref, since_ns, uncov):
    st = ctx.state
    vs, us = np.nonzero(blob)
    src = d
    if kind == "removed" and np.isfinite(ref[blob]).any():
        src = ref
    z = float(np.nanmedian(src[blob]))
    if not math.isfinite(z):
        z = float(np.nanmedian(st["bdepth"][blob]))
    u = float(us.mean()) * STEP + (STEP - 1) / 2.0
    v = float(vs.mean()) * STEP + (STEP - 1) / 2.0
    w = ctx.deproject(u, v, z)
    h = float(np.nanmedian(((st["bdepth"] - src) * st["dz"])[blob]))
    area_px = int(blob.sum()) * STEP * STEP
    fields = [("u", round(u, 1)), ("v", round(v, 1)), ("x", round(w["x"], 3)),
              ("y", round(w["y"], 3)), ("z", round(w["z"], 3)), ("height_m", round(h, 3)),
              ("side_m", round(math.sqrt(area_px) / st["ppm"], 3)),
              ("since_wall_ns", int(since_ns)), ("uncov", int(uncov))]
    detail = '{"kind":"' + kind + '"' + "".join(',"' + k + '":' + str(x) for k, x in fields) + "}"
    return w, {"value": float(area_px), "detail": detail}

def check(ctx):
    d = ctx.depth()
    cam = ctx.camera()
    if d is None or cam is None:
        return None
    d = d[::STEP, ::STEP]
    st = ctx.state
    now = ctx.now_wall_ns()
    if "bdepth" not in st or st["bdepth"].shape != d.shape:
        setup(ctx, cam, d.shape)
        st["ref"] = np.full(d.shape, np.nan, dtype=np.float32)
        st["age"] = np.zeros(d.shape, dtype=np.int16)
        st["stable"] = np.zeros(d.shape, dtype=np.int16)
        st["since"] = np.zeros(d.shape, dtype=np.int64)
        st["mticks"] = 0
        st["msince"] = 0
        st["prev"] = d.copy()
    track(ctx)
    valid = np.isfinite(d)
    hgt = (st["bdepth"] - d) * st["dz"]
    mask = rmask(ctx, np.where(valid, hgt, 0.0), valid)
    ref = st["ref"]
    known = np.isfinite(ref)
    rise = np.where(known, ref - d, np.where(valid, np.maximum(hgt, 0.0), 0.0))
    if "init" not in st:
        st["init"] = True
        ref[valid & ~mask] = d[valid & ~mask]
        st["prev"] = d.copy()
        return None
    prev = st["prev"]
    still = valid & np.isfinite(prev) & (np.abs(d - prev) < HMIN)
    st["stable"] = np.where(still, st["stable"] + 1, 0)
    st["prev"] = d.copy()
    changed = valid & ~mask & (np.abs(rise) > HMIN)
    fill = valid & ~mask & ~known & (st["stable"] >= PER) & ~changed
    ref[fill] = d[fill]
    moving = valid & ~dilate(mask, MOVE_GAP) & ~still & (rise > HMIN)
    st["age"] = np.where(changed & ~mask, st["age"] + 1, 0)
    st["since"] = np.where(st["age"] == 1, now, st["since"])
    hit = st["age"] >= PER
    if hit.sum() >= MINC:
        flat = np.argmax(np.where(hit, st["age"], 0))
        seed = np.zeros(hit.shape, dtype=bool)
        seed[flat // hit.shape[1], flat % hit.shape[1]] = True
        blob = connected(seed, hit)
        if blob.sum() >= MINC:
            kind = "new_static" if float(np.nanmedian(rise[blob])) > 0 else "removed"
            uncov = float(np.mean(~known[blob])) > 0.5
            w, report = rep(ctx, kind, blob, d, ref, st["since"][blob].min(), uncov)
            ref[blob] = d[blob]
            st["age"][blob] = 0
            struct = dilate(mask | (valid & (hgt > ARMH)), RGAP)
            if uncov and (bool((struct & blob).any()) or w["y"] < ZY):
                return None
            if explained(ctx, w, kind) is None:
                return report
        else:
            st["age"][blob] = 0
    if moving.sum() >= MCELLS:
        st["mticks"] += 1
        if st["mticks"] == 1:
            st["msince"] = now
        if st["mticks"] >= MPER:
            w, report = rep(ctx, "moving", moving, d, ref, st["msince"], False)
            st["mticks"] = 0
            if explained(ctx, w, "moving") is None:
                return report
    else:
        st["mticks"] = 0
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
        if oid.endswith("_placement") or oid.endswith("_reorient") or oid.endswith("_hole"):
            continue
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
        "rate_hz": 2.0,
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
