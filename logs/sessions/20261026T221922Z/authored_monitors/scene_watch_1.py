def seg_dist(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    L2 = vx * vx + vy * vy
    if L2 < 1e-9:
        return np.sqrt((px - ax) ** 2 + (py - ay) ** 2)
    t = ((px - ax) * vx + (py - ay) * vy) / L2
    t = np.clip(t, 0.0, 1.0)
    cx = ax + t * vx
    cy = ay + t * vy
    return np.sqrt((px - cx) ** 2 + (py - cy) ** 2)

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    ds = d[::2, ::2]
    if "base" not in ctx.state:
        ctx.state["base"] = np.array(ds, dtype=np.float32)
        ctx.state["ticks"] = 0
        return None
    ctx.state["ticks"] = ctx.state["ticks"] + 1
    diff = ctx.state["base"] - ds
    mask = diff > ctx.value("near_change_m")
    vs, us = np.nonzero(mask)
    n0 = int(len(vs))
    if n0 < ctx.value("pix_trigger"):
        return None
    zs = ds[vs, us]
    P = ctx.deproject(us * 2, vs * 2, zs)
    px = P[:, 0]
    py = P[:, 1]
    pz = P[:, 2]
    keep = pz > 0.01
    tcp = ctx.frame_pose("tcp")
    bx = ctx.value("base_x")
    by = ctx.value("base_y")
    keep = keep & (np.sqrt((px - bx) ** 2 + (py - by) ** 2) > ctx.value("base_radius_m"))
    if tcp is not None:
        tx = tcp["pose"]["position"]["x"]
        ty = tcp["pose"]["position"]["y"]
        keep = keep & (seg_dist(px, py, bx, by, tx, ty) > ctx.value("arm_band_m"))
        keep = keep & (np.sqrt((px - tx) ** 2 + (py - ty) ** 2) > ctx.value("tcp_radius_m"))
    for oid in ctx.object_ids():
        if oid.endswith("_placement") or oid.endswith("_hole") or oid.endswith("_reorient"):
            continue
        op = ctx.object_pose(oid)
        if op is None:
            continue
        ox = op["pose"]["position"]["x"]
        oy = op["pose"]["position"]["y"]
        keep = keep & (np.sqrt((px - ox) ** 2 + (py - oy) ** 2) > ctx.value("obj_radius_m"))
    n = int(np.sum(keep))
    if n > ctx.value("pix_trigger"):
        cx = float(np.mean(px[keep]))
        cy = float(np.mean(py[keep]))
        cz = float(np.max(pz[keep]))
        return {"value": float(n), "detail": "unexplained near-change pixels=%d centroid world x=%.3f y=%.3f maxz=%.3f" % (n, cx, cy, cz)}
    return None
