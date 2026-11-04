def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    diff = np.abs(d - ctx.state["ref"])
    diff = diff[diff == diff]
    if diff.size == 0:
        return None
    changed = float(np.mean(diff > ctx.value("chg")))
    if changed > ctx.value("frac"):
        ctx.state["ref"] = d.copy()
        return {"value": changed, "detail": "workspace depth scene changed beyond baseline over a large fraction of pixels"}
    return None
