def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    ref = ctx.state["ref"]
    if ref.shape != d.shape:
        ctx.state["ref"] = d.copy()
        return None
    diff = np.abs(d - ref)
    valid = ~np.isnan(diff)
    if not np.any(valid):
        return None
    changed_frac = float(np.sum(valid & (diff > 0.03)) / np.sum(valid))
    if changed_frac > ctx.value("scene_change_frac"):
        ctx.state["ref"] = d.copy()
        return {"value": changed_frac, "detail": "workspace depth changed significantly over a broad region"}
    return None
