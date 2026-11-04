def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    e = abs(j["effort"])
    thr = ctx.value("max_clamp_effort")
    if e > thr:
        return {"value": e, "detail": "inverted_u_yellow clamp effort " + str(e) + " exceeds limit " + str(thr)}
    return None
