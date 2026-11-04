def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    if ctx.held_object() != OBJ:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint |effort| %.2f exceeds max_clamp_effort while holding %s" % (e, OBJ)}
    return None

OBJ = "inverted_u_yellow"
