def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    e = j["effort"]
    ctx.state["last_effort"] = e
    if e > ctx.value("max_effort"):
        return {"value": float(e), "detail": "finger_joint effort exceeded inverted_u_yellow max_clamp_effort"}
    return None
