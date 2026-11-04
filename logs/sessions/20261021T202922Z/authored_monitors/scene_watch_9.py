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
    changed = float(np.nanmean(diff[valid]))
    if changed > ctx.value("change_m"):
        return {"value": changed, "detail": "workspace depth image changed beyond scan threshold"}
    return None
