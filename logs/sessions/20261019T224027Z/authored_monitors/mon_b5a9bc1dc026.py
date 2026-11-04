def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    ref = ctx.state["ref"]
    diff = np.abs(d - ref)
    valid = ~np.isnan(diff)
    if not np.any(valid):
        return None
    changed_frac = float(np.mean(valid & (diff > ctx.value("depth_change_m"))))
    if changed_frac > ctx.value("area_frac"):
        return {"value": changed_frac, "detail": "large depth change fraction in scene vs reference frame"}
    return None
