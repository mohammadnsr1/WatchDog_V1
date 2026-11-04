def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    eff = abs(j["effort"])
    limit = ctx.value("max_clamp_effort")
    if eff > limit:
        return {"value": eff, "detail": "finger_joint effort exceeded inverted_u_yellow max_clamp_effort while held"}
    return None
