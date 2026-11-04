def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    e = abs(j["effort"])
    thr = ctx.value("max_clamp_effort")
    if e > thr:
        return {"value": e, "detail": "finger_joint effort " + str(e) + " exceeded max_clamp_effort while holding inverted_u_yellow"}
    return None