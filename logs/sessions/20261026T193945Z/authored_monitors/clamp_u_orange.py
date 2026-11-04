def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if "armed" not in ctx.state:
        ctx.state["armed"] = True
        ctx.state["effort_at_arm"] = e
        return None
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort " + str(round(e, 3)) + " exceeds max_clamp_effort"}
    return None
