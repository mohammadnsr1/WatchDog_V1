def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    eff = abs(j["effort"])
    if eff > ctx.value("max_clamp_effort"):
        return {"value": eff, "detail": "finger_joint effort exceeded u_orange's tolerated clamp effort"}
    return None
