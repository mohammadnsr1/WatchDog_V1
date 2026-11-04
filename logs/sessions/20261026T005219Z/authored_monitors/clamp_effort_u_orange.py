def check(ctx):
    lim = ctx.value("max_clamp_effort")
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if "seen" not in ctx.state:
        ctx.state["seen"] = e
        return None
    if e > lim:
        return {"value": float(e), "detail": "finger_joint effort exceeded u_orange max_clamp_effort"}
    return None
