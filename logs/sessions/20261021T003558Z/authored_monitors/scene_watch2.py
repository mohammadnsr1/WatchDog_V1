def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    diff = np.abs(d - ctx.state["ref"])
    changed = np.nanmean(diff)
    if changed > ctx.value("change_m"):
        return {"value": float(changed), "detail": "workspace depth image changed beyond baseline, possible unexpected object/event"}
    return None