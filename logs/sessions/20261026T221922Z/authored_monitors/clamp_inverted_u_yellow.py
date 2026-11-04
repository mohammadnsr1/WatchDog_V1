def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_effort"):
        return {"value": e, "detail": "finger_joint |effort| %.3f exceeds max_clamp_effort while holding inverted_u_yellow" % e}
    return None
