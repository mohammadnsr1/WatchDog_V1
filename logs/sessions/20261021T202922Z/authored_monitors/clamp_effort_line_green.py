def check(ctx):
    held = ctx.held_object()
    if held != "line_green":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded max_clamp_effort while holding line_green"}
    return None
