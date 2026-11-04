def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    diff = np.abs(d - ctx.state["ref"])
    valid = ~np.isnan(diff)
    if not np.any(valid):
        return None
    changed = float(np.nanmean(diff))
    if changed > ctx.value("scene_change_m"):
        ctx.state["ref"] = d.copy()
        return {"value": changed, "detail": "workspace depth image changed beyond baseline noise"}
    return None
