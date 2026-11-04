def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    ctx.state["last_effort"] = j["effort"]
    if ctx.held_object() != "inverted_u_yellow":
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded inverted_u_yellow max_clamp_effort"}
    return None
