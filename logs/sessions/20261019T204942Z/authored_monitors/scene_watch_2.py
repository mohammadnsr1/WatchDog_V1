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
    changed_frac = float(np.sum(valid & (diff > ctx.value("depth_change_m"))) / np.sum(valid))
    if changed_frac > ctx.value("scene_change_frac"):
        ctx.state["ref"] = d.copy()
        return {"value": changed_frac, "detail": "workspace depth image changed over " + str(changed_frac) + " of valid pixels"}
    ctx.state["ref"] = d.copy()
    return None
