def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint instantaneous effort exceeded object's tolerated clamp effort"}
    return None
