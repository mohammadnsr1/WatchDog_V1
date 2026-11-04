
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
