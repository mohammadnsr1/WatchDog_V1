def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    e = abs(j["effort"])
    ctx.state["last_effort"] = e
    if e > ctx.value("max_effort"):
        return {"value": e, "detail": "finger_joint effort " + str(round(e, 2)) + " exceeds u_orange limit"}
    return None
