STRIDE = 4
def check(ctx):
    d = ctx.depth()
    tcp = ctx.frame_pose("tcp")
    if d is None or tcp is None:
        return None
    if "uu" not in ctx.state:
        h = d.shape[0]
        w = d.shape[1]
        g = np.mgrid[0:h:STRIDE, 0:w:STRIDE]
        ctx.state["vv"] = g[0].ravel().astype(np.int64)
        ctx.state["uu"] = g[1].ravel().astype(np.int64)
    uu = ctx.state["uu"]
    vv = ctx.state["vv"]
    z = np.asarray(d[vv, uu], dtype=np.float64)
    valid = np.isfinite(z)
    zz = np.where(valid, z, 1.0)
    P = np.asarray(ctx.deproject(uu.astype(np.float64), vv.astype(np.float64), zz), dtype=np.float64)
    tp = tcp["pose"]["position"]
    rxy = np.sqrt((P[:, 0] - tp["x"]) ** 2 + (P[:, 1] - tp["y"]) ** 2)
    bench = valid & (P[:, 2] < ctx.value("bench_z_max_m")) & (P[:, 2] > -0.05)
    far = rxy > ctx.value("tcp_excl_radius_m")
    if "ref" not in ctx.state:
        ctx.state["ref"] = np.where(bench, P[:, 2], np.nan)
        ctx.state["n_bench"] = int(np.sum(bench))
        return None
    ref = ctx.state["ref"]
    hasref = np.isfinite(ref)
    diff = np.abs(np.where(hasref, P[:, 2] - np.where(hasref, ref, 0.0), 0.0))
    changed = bench & far & hasref & (diff > ctx.value("dz_m"))
    n = int(np.sum(changed))
    upd = bench & ((~far) | (~hasref))
    ctx.state["ref"] = np.where(upd, P[:, 2], ref)
    ctx.state["last_n"] = n
    if n > ctx.value("min_px"):
        idx = np.nonzero(changed)[0]
        cx = float(np.mean(P[idx, 0]))
        cy = float(np.mean(P[idx, 1]))
        cz = float(np.max(P[idx, 2]))
        cu = int(np.mean(uu[idx]))
        cv = int(np.mean(vv[idx]))
        return {"value": float(n), "detail": "unexplained bench-level depth change: %d sampled px, centroid world (%.3f, %.3f), max z %.3f, pixel (%d, %d), tcp (%.3f, %.3f)" % (n, cx, cy, cz, cu, cv, tp["x"], tp["y"])}
    return None
