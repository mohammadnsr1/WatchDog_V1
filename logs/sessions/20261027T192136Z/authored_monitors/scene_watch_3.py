LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tcp"]
OBJS = ["u_brown", "u_orange", "line_green", "inverted_u_yellow", "base1", "base2", "u_green", "inverted_u_brown", "hex_blue", "hex_red"]

def qrot(q, v):
    x, y, z, w = q
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)]

def project(cam, p):
    t = cam["world_T_camera"]["translation"]
    r = cam["world_T_camera"]["rotation_xyzw"]
    rinv = [-r[0], -r[1], -r[2], r[3]]
    c = qrot(rinv, [p[0] - t[0], p[1] - t[1], p[2] - t[2]])
    if c[2] <= 0.05:
        return None
    K = cam["K"]
    return (K[0][0] * c[0] / c[2] + K[0][2], K[1][1] * c[1] / c[2] + K[1][2], c[2], K[0][0])

def link_pts(ctx):
    pts = []
    for l in LINKS:
        try:
            tr = ctx.transform("world", l)["translation"]
            pts.append([tr["x"], tr["y"], tr["z"]])
        except Exception:
            pass
    return pts

def obj_pts(ctx):
    pts = []
    for o in OBJS:
        p = ctx.object_pose(o)
        if p is not None:
            q = p["pose"]["position"]
            pts.append([q["x"], q["y"], q["z"]])
    return pts

def paint(mask, cam, segs, pts, rad_m, rad_pt, min_px, vv, uu):
    for a, b in segs:
        pa = project(cam, a)
        pb = project(cam, b)
        if pa is None or pb is None:
            continue
        ra = max(min_px, pa[3] * rad_m / pa[2])
        rb = max(min_px, pb[3] * rad_m / pb[2])
        dx = pb[0] - pa[0]
        dy = pb[1] - pa[1]
        L2 = dx * dx + dy * dy + 1e-9
        t = np.clip(((uu - pa[0]) * dx + (vv - pa[1]) * dy) / L2, 0.0, 1.0)
        d2 = (uu - pa[0] - t * dx) ** 2 + (vv - pa[1] - t * dy) ** 2
        r = ra + t * (rb - ra)
        mask |= d2 < r * r
    for p in pts:
        pp = project(cam, p)
        if pp is None:
            continue
        mask |= ((uu - pp[0]) ** 2 + (vv - pp[1]) ** 2) < rad_pt * rad_pt
    return mask

def check(ctx):
    d = ctx.depth()
    cam = ctx.camera()
    if d is None or cam is None:
        return None
    s = 4
    ds = d[::s, ::s]
    h, w = ds.shape
    vv, uu = np.mgrid[0:h, 0:w]
    vv = vv * s
    uu = uu * s
    lp = link_pts(ctx)
    if "base" not in ctx.state:
        ctx.state["base"] = ds.copy()
        ctx.state["link0"] = lp
        ctx.state["obj0"] = obj_pts(ctx)
        ctx.state["hist"] = [lp]
        return None
    hist = ctx.state["hist"] + [lp]
    hist = hist[-4:]
    ctx.state["hist"] = hist
    segs = []
    for chain in hist + [ctx.state["link0"]]:
        segs = segs + [(chain[i], chain[i + 1]) for i in range(len(chain) - 1)]
    mask = np.zeros((h, w), dtype=bool)
    mask = paint(mask, cam, segs, obj_pts(ctx) + ctx.state["obj0"], ctx.value("arm_mask_m"), ctx.value("obj_mask_px"), ctx.value("arm_min_px"), vv, uu)
    diff = np.abs(ds - ctx.state["base"])
    ch = (diff > ctx.value("depth_change_m")) & (~mask)
    n = int(np.count_nonzero(ch))
    ctx.state["last_n"] = n
    if n > ctx.value("min_changed_px"):
        ys, xs = np.nonzero(ch)
        u = float(np.median(xs)) * s
        v = float(np.median(ys)) * s
        return {"value": float(n), "detail": "unexplained depth change, %d sampled px, centroid pixel u=%.0f v=%.0f" % (n, u, v)}
    return None
