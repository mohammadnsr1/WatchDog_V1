def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    ref = ctx.state["ref"]
    diff = np.abs(ref - d)
    valid = ~np.isnan(diff)
    cnt = int(np.sum((diff > ctx.value("change_m")) & valid))
    ctx.state["ref"] = d.copy()
    if cnt > ctx.value("min_px"):
        return {"value": float(cnt), "detail": "scene depth changed at " + str(cnt) + " pixels vs previous frame"}
    return None
