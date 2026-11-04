def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    mean = float(np.nanmean(d))
    if "ref" not in ctx.state:
        ctx.state["ref"] = mean
        return None
    change = abs(mean - ctx.state["ref"])
    if change > ctx.value("scene_change_m"):
        return {"value": change, "detail": "workspace depth scene changed beyond the agent-chosen look-trigger threshold"}
    return None
