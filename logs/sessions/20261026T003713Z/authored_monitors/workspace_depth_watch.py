def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    diff = np.abs(d - ctx.state["ref"])
    valid = np.isfinite(diff)
    n = int(valid.sum())
    if n == 0:
        return None
    frac = float((diff[valid] > ctx.value("delta_m")).sum()) / n
    if frac > ctx.value("frac"):
        return {"value": frac, "detail": "large depth change across workspace"}
    return None
