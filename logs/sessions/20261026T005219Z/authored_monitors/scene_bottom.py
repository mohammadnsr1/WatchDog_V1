def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    h = d.shape[0]
    strip = d[h - h // 8:, :].ravel()
    if "ref" not in ctx.state:
        ctx.state["ref"] = strip.copy()
        return None
    diff = np.abs(strip - ctx.state["ref"])
    frac = float(np.nanmean(diff > ctx.value("change_m")))
    if frac > ctx.value("frac"):
        return {"value": frac, "detail": "depth change in bottom (operator-side) strip: possible entry"}
    return None
