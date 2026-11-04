def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    eff = abs(j["effort"])
    limit = ctx.value("max_clamp_effort")
    if eff > limit:
        return {"value": eff, "detail": "finger_joint effort exceeded u_brown max_clamp_effort while held"}
    return None
