LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tcp"]

def rotmat(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])

def project(ctx, cam, p):
    t = np.array(cam["world_T_camera"]["translation"])
    R = rotmat(cam["world_T_camera"]["rotation_xyzw"])
    pc = R.T.dot(np.array(p) - t)
    if pc[2] <= 0.05:
        return None
    K = cam["K"]
    return (K[0][0] * pc[0] / pc[2] + K[0][2], K[1][1] * pc[1] / pc[2] + K[1][2])

def arm_mask(ctx, cam, h, w):
    pts = []
    for ln in LINKS:
        try:
            tr = ctx.transform("world", ln)["translation"]
        except:
            continue
        uv = project(ctx, cam, [tr["x"], tr["y"], tr["z"]])
        if uv is not None:
            pts.append(uv)
    vv, uu = np.mgrid[0:h, 0:w]
    uu = uu.astype(np.float32)
    vv = vv.astype(np.float32)
    mask = np.zeros((h, w), dtype=bool)
    rad = ctx.value("mask_radius_px")
    for i in range(len(pts)):
        a = pts[i]
        b = pts[i + 1] if i + 1 < len(pts) else pts[i]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        L2 = dx * dx + dy * dy
        if L2 < 1e-6:
            d2 = (uu - a[0]) ** 2 + (vv - a[1]) ** 2
        else:
            s = np.clip(((uu - a[0]) * dx + (vv - a[1]) * dy) / L2, 0.0, 1.0)
            d2 = (uu - a[0] - s * dx) ** 2 + (vv - a[1] - s * dy) ** 2
        mask = mask | (d2 < rad * rad)
    return mask, len(pts)

def check(ctx):
    d = ctx.depth()
    cam = ctx.camera()
    if d is None or cam is None:
        return None
    d = np.where(np.isfinite(d), d, 0.0).astype(np.float32)
    h = d.shape[0]
    w = d.shape[1]
    mask, n = arm_mask(ctx, cam, h, w)
    ctx.state["links_projected"] = n
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        ctx.state["prev"] = np.zeros((h, w), dtype=bool)
        return None
    ref = ctx.state["ref"]
    valid = (d > 0) & (ref > 0)
    closer = valid & (~mask) & ((ref - d) > ctx.value("closer_m"))
    both = closer & ctx.state["prev"]
    cnt = int(both.sum())
    ctx.state["prev"] = closer
    ctx.state["ref"] = np.where(mask, d, np.maximum(ref, d))
    ctx.state["last_count"] = cnt
    if cnt > ctx.value("min_pixels"):
        ys, xs = np.nonzero(both)
        return {"value": float(cnt), "detail": "persistent closer-than-reference region of " + str(cnt) + " px outside arm mask, centroid u=" + str(int(xs.mean())) + " v=" + str(int(ys.mean()))}
    return None
