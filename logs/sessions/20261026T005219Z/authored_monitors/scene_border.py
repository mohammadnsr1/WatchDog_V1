def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    h = d.shape[0]
    w = d.shape[1]
    border = np.concatenate([d[0:h // 8, :].ravel(), d[:, 0:w // 10].ravel(), d[:, w - w // 10:].ravel()])
    if "ref" not in ctx.state:
        ctx.state["ref"] = border.copy()
        return None
    diff = np.abs(border - ctx.state["ref"])
    frac = float(np.nanmean(diff > ctx.value("change_m")))
    if frac > ctx.value("frac"):
        return {"value": frac, "detail": "depth change at workspace border (possible entry)"}
    return None
