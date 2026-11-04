def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    h = d.shape[0]
    w = d.shape[1]
    b = int(ctx.value("border_px"))
    strips = np.concatenate([d[h - b:h, :].ravel(), d[h // 3:h, w - b:w].ravel()])
    if "ref" not in ctx.state:
        ctx.state["ref"] = strips.copy()
        return None
    diff = np.abs(strips - ctx.state["ref"])
    valid = np.isfinite(diff)
    n = int(np.sum(valid))
    if n == 0:
        return None
    frac = float(np.sum(diff[valid] > ctx.value("depth_change_m"))) / n
    if frac > ctx.value("changed_frac"):
        return {"value": frac, "detail": "depth changed on bottom/right workspace border strips (possible entry into workspace)"}
    return None
