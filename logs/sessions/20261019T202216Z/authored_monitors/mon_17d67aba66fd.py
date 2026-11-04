OBJ = "line_green"
def check(ctx):
    held = ctx.held_object()
    if held != OBJ:
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = j["effort"]
    if abs(e) > ctx.value("max_effort"):
        return {"value": abs(e), "detail": "finger_joint effort exceeded max_clamp_effort while holding line_green"}
    return None