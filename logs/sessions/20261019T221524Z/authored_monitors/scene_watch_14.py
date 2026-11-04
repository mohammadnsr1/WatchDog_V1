def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        ctx.state["checks"] = 0
        return None
    ctx.state["checks"] = ctx.state.get("checks", 0) + 1
    ref = ctx.state["ref"]
    diff = np.abs(d - ref)
    valid = ~np.isnan(diff)
    if not np.any(valid):
        return None
    changed_frac = float(np.mean(valid & (diff > 0.03)))
    if changed_frac > ctx.value("scene_change_frac"):
        ctx.state["ref"] = d.copy()
        return {"value": changed_frac, "detail": "depth image changed over " + str(changed_frac) + " of valid pixels vs reference"}
    return None