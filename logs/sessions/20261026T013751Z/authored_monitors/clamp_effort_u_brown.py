def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    ctx.state["last_effort"] = e
    if e > ctx.value("max_clamp_effort"):
        return {"value": float(e), "detail": "finger_joint effort " + str(round(e, 3)) + " exceeds u_brown max_clamp_effort"}
    return None
