def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_effort"):
        return {"value": e, "detail": "finger_joint |effort| %.3f exceeds max_clamp_effort while holding line_green" % e}
    return None
