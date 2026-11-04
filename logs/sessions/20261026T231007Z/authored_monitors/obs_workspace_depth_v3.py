LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tcp"]
TASK = ["u_brown", "u_orange", "line_green", "inverted_u_yellow", "base1"]
FRAMES = ["u_brown_placement", "u_orange_placement", "line_green_placement", "inverted_u_yellow_placement"]
STEP = 4

def link_pts(ctx):
    pts = []
    for n in LINKS:
        t = ctx.transform("world", n)["translation"]
        pts.append([t["x"], t["y"], t["z"]])
    return np.array(pts)

def near_segments(P, pts, r):
    m = np.zeros(P.shape[0], dtype=bool)
    for i in range(len(pts) - 1):
        a = pts[i]
        ab = pts[i + 1] - a
        L = float(np.dot(ab, ab))
        if L < 1e-9:
            t = np.zeros(P.shape[0])
        else:
            t = np.clip((P - a) @ ab / L, 0.0, 1.0)
        c = a + t[:, None] * ab
        m = m | (np.linalg.norm(P - c, axis=1) < r)
    return m

def task_xy(ctx):
    xy = []
    for o in TASK:
        p = ctx.object_pose(o)
        if p is not None:
            q = p["pose"]["position"]
            xy.append([q["x"], q["y"]])
    for f in FRAMES:
        p = ctx.frame_pose(f)
        if p is not None:
            q = p["pose"]["position"]
            xy.append([q["x"], q["y"]])
    return xy

def near_xy(P, xy, r):
    m = np.zeros(P.shape[0], dtype=bool)
    for c in xy:
        m = m | (np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < r)
    return m

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    h = d.shape[0]
    w = d.shape[1]
    vv, uu = np.mgrid[0:h:STEP, 0:w:STEP]
    dz = d[0:h:STEP, 0:w:STEP]
    if "base" not in ctx.state:
        ctx.state["base"] = dz.copy()
        ctx.state["base_links"] = link_pts(ctx)
        ctx.state["init_xy"] = [[0.5685720443725586, -0.3336311876773834], [0.6740754842758179, -0.34971562027931213], [0.610243558883667, -0.22728823125362396], [0.5303542613983154, -0.06459744274616241]]
        return None
    b = ctx.state["base"]
    ok = np.isfinite(dz) & np.isfinite(b)
    diff = np.zeros(dz.shape)
    diff[ok] = np.abs(dz[ok] - b[ok])
    ch = ok & (diff > ctx.value("depth_change_m"))
    if not ch.any():
        ctx.state["last_count"] = 0
        return None
    u = uu[ch].astype(float)
    v = vv[ch].astype(float)
    Pc = ctx.deproject(u, v, dz[ch])
    Pb = ctx.deproject(u, v, b[ch])
    r = ctx.value("arm_mask_radius_m")
    rt = ctx.value("task_mask_radius_m")
    m = near_segments(Pc, link_pts(ctx), r) | near_segments(Pb, ctx.state["base_links"], r)
    xy = task_xy(ctx) + ctx.state["init_xy"]
    m = m | near_xy(Pc, xy, rt) | near_xy(Pb, xy, rt)
    keep = ~m
    n = int(keep.sum())
    ctx.state["last_count"] = n
    if n > ctx.value("min_changed_px"):
        cx = float(np.mean(Pc[keep, 0]))
        cy = float(np.mean(Pc[keep, 1]))
        cz = float(np.max(Pc[keep, 2]))
        return {"value": float(n), "detail": "unexplained depth change: " + str(n) + " subsampled px, centroid world x=" + str(round(cx, 3)) + " y=" + str(round(cy, 3)) + " max z=" + str(round(cz, 3))}
    return None
