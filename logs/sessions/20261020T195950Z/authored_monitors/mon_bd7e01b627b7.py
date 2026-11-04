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
    changed_frac = float(np.mean(valid & (diff > 0.02)))
    if changed_frac > ctx.value("scene_change_frac"):
        return {"value": changed_frac, "detail": "workspace depth changed over " + str(changed_frac) + " of pixels vs reference"}
    return None
