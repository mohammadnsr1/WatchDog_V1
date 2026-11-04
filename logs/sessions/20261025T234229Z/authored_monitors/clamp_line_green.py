def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    ctx.state["last_effort"] = j["effort"]
    if ctx.held_object() != "line_green":
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded line_green max_clamp_effort"}
    return None
