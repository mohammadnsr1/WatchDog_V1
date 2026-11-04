def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    limit = ctx.value("max_clamp_effort")
    e = abs(j["effort"])
    if e > limit:
        return {"value": e, "detail": "finger_joint effort " + str(e) + " exceeds line_green max_clamp_effort"}
    return None
