OBJ = "u_orange"
def check(ctx):
    if ctx.held_object() != OBJ:
        return None
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    e = j["effort"]
    if e > ctx.value("max_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded max_clamp_effort while holding u_orange"}
    return None
