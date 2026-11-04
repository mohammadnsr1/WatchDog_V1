def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    eff = abs(j["effort"])
    limit = ctx.value("max_clamp_effort")
    if eff > limit:
        return {"value": eff, "detail": "clamp effort exceeded max_clamp_effort on u_brown"}
    return None