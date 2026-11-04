OBJ = "u_orange"
def check(ctx):
    if ctx.held_object() != OBJ:
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    eff = abs(j["effort"])
    if eff > ctx.value("max_clamp_effort"):
        return {"value": eff, "detail": "finger_joint effort exceeded max_clamp_effort while holding " + OBJ}
    return None
