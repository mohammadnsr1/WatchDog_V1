def check(ctx):
    if ctx.held_object() != "u_orange":
        return None
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    e = abs(j["effort"])
    thr = ctx.value("max_clamp_effort")
    if e > thr:
        return {"value": e, "detail": "u_orange clamp effort " + str(e) + " exceeded max " + str(thr)}
    return None
