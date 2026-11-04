def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    eff = abs(j["effort"])
    lim = ctx.value("lim")
    if eff > lim:
        return {"value": eff, "detail": "finger_joint effort exceeded u_orange max_clamp_effort"}
    return None
