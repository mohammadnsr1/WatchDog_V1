OBJS = ["u_green", "base2", "hex_blue", "hex_red", "inverted_u_brown", "base1", "u_brown", "u_orange", "inverted_u_yellow", "line_green"]

def xy_list(ctx):
    out = []
    for oid in OBJS:
        p = ctx.object_pose(oid)
        if p is not None:
            q = p["pose"]["position"]
            out.append((q["x"], q["y"]))
    return out

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = np.array(d, dtype=np.float32)
        ctx.state["ref_xy"] = xy_list(ctx)
        return None
    ref = ctx.state["ref"]
    if ref.shape != d.shape:
        return None
    step = 2
    dd = d[::step, ::step]
    rr = ref[::step, ::step]
    nearest = np.fmin(dd, rr)
    ch = np.abs(dd - rr) > ctx.value("depth_change_m")
    ch = ch & np.isfinite(dd) & np.isfinite(rr)
    vs, us = np.nonzero(ch)
    if len(vs) == 0:
        ctx.state["last_count"] = 0
        return None
    z = nearest[vs, us]
    pts = ctx.deproject(us * step, vs * step, z)
    keep = (pts[:, 2] > 0.005) & (pts[:, 2] < ctx.value("max_z_m"))
    t = ctx.frame_pose("tcp")
    if t is not None:
        tp = t["pose"]["position"]
        keep = keep & (np.hypot(pts[:, 0] - tp["x"], pts[:, 1] - tp["y"]) > ctx.value("tcp_excl_m"))
    for (x, y) in ctx.state["ref_xy"] + xy_list(ctx):
        keep = keep & (np.hypot(pts[:, 0] - x, pts[:, 1] - y) > ctx.value("obj_excl_m"))
    n = int(np.count_nonzero(keep))
    ctx.state["last_count"] = n
    if n > ctx.value("pixel_count"):
        k = np.nonzero(keep)[0]
        c = pts[k].mean(axis=0)
        return {"value": float(n), "detail": "unexplained depth change: %d px (stride 2) centred near world (%.2f, %.2f, %.2f)" % (n, c[0], c[1], c[2])}
    return None
