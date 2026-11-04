def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[160:198, 356:396]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        return None
    diff = np.abs(roi - ctx.state["ref"])
    valid = np.isfinite(diff)
    n = int(np.sum(valid))
    if n == 0:
        return None
    frac = float(np.sum(diff[valid] > ctx.value("depth_change_m"))) / n
    if frac > ctx.value("changed_frac"):
        return {"value": frac, "detail": "depth changed in the unlisted-cube ROI: cube moved, left, or something passed over it"}
    return None
