def check(ctx):
    held = ctx.held_object()
    if held != "u_brown":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    eff = abs(j["effort"])
    limit = ctx.value("max_clamp_effort")
    if eff > limit:
        return {"value": eff, "detail": "finger_joint effort exceeded limit while holding u_brown"}
    return None
