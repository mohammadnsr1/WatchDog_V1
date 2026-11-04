def check(ctx):
    held = ctx.held_object()
    if held != "inverted_u_yellow":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    eff = abs(j["effort"])
    limit = ctx.value("max_clamp_effort")
    if eff > limit:
        return {"value": eff, "detail": "finger_joint effort " + str(eff) + " exceeded max_clamp_effort " + str(limit) + " while holding inverted_u_yellow"}
    return None