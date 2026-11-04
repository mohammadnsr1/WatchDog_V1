def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def check(ctx):
    d = ctx.depth()
    cam = ctx.camera()
    if d is None or cam is None:
        return None
    s = 2
    ds = d[::s, ::s]
    h, w = ds.shape
    vv, uu = np.mgrid[0:h, 0:w]
    uu = uu * s
    vv = vv * s
    pts = ctx.deproject(uu.ravel(), vv.ravel(), ds.ravel())
    z = pts[:, 2].reshape(h, w)
    yw = pts[:, 1].reshape(h, w)
    low = np.isfinite(z) & (z < ctx.value("bench_band_m")) & (yw > ctx.value("rail_strip_y_m"))
    if "ref_d" not in ctx.state:
        ctx.state["ref_d"] = ds.copy()
        ctx.state["ref_low"] = low.copy()
        ctx.state["xy"] = pts[:, 0:2].reshape(h, w, 2).copy()
        ctx.state["ref_low_px"] = int(low.sum())
        return None
    K = cam["K"]
    t = np.array(cam["world_T_camera"]["translation"])
    R = qmat(cam["world_T_camera"]["rotation_xyzw"])
    mask = np.zeros((h, w), dtype=bool)
    tcp = ctx.frame_pose("tcp")
    if tcp is not None:
        p = tcp["pose"]["position"]
        pc = R.T.dot(np.array([p["x"], p["y"], p["z"]]) - t)
        if pc[2] > 0.05:
            tu = K[0][0] * pc[0] / pc[2] + K[0][2]
            tv = K[1][1] * pc[1] / pc[2] + K[1][2]
            rad = K[0][0] * ctx.value("tcp_mask_radius_m") / pc[2]
            mask = ((uu - tu) ** 2 + (vv - tv) ** 2) < rad * rad
    diff = np.abs(ds - ctx.state["ref_d"])
    changed = np.isfinite(diff) & (diff > ctx.value("depth_change_m")) & low & ctx.state["ref_low"] & (~mask)
    n = int(changed.sum())
    if n > ctx.value("min_pixels"):
        xy = ctx.state["xy"][changed]
        cx = float(np.mean(xy[:, 0]))
        cy = float(np.mean(xy[:, 1]))
        return {"value": float(n), "detail": "depth change near bench outside TCP mask: %d px (half-res), centroid world x=%.3f y=%.3f, extent x[%.3f,%.3f] y[%.3f,%.3f]" % (n, cx, cy, float(xy[:, 0].min()), float(xy[:, 0].max()), float(xy[:, 1].min()), float(xy[:, 1].max()))}
    return None
