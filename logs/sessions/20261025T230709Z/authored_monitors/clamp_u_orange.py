def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if "max_seen" not in ctx.state:
        ctx.state["max_seen"] = e
        ctx.state["limit"] = ctx.value("max_clamp_effort")
    elif e > ctx.state["max_seen"]:
        ctx.state["max_seen"] = e
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded max_clamp_effort"}
    return None
