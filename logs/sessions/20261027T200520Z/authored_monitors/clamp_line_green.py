OBJ = "line_green"
JOINT = "finger_joint"

def check(ctx):
    j = ctx.joint(JOINT)
    if j is None:
        return None
    ctx.state["last_effort"] = j["effort"]
    if ctx.held_object() != OBJ:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint |effort| %.3f exceeds max_clamp_effort %.1f while holding %s" % (e, ctx.value("max_clamp_effort"), OBJ)}
    return None
