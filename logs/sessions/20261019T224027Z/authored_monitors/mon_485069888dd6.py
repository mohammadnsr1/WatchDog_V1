def check(ctx):
    held = ctx.held_object()
    if held != "u_orange":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    eff = abs(j["effort"])
    if eff > ctx.value("max_clamp_effort"):
        return {"value": float(eff), "detail": "finger_joint effort exceeded u_orange's max_clamp_effort while held"}
    return None
