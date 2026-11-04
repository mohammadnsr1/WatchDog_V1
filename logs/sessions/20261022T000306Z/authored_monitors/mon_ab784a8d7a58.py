def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    limit = ctx.value("max_clamp_effort")
    e = abs(j["effort"])
    if e > limit:
        return {"value": e, "detail": "finger_joint effort exceeded u_orange max_clamp_effort"}
    return None
