def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    ctx.state["last_effort"] = j["effort"]
    if ctx.held_object() != "u_orange":
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded u_orange max_clamp_effort"}
    return None
