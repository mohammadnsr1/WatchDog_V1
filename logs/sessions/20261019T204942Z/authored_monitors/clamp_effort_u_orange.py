def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded max_clamp_effort while holding u_orange"}
    return None
