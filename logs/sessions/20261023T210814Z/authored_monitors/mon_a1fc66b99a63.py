def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded inverted_u_yellow max_clamp_effort"}
    return None
