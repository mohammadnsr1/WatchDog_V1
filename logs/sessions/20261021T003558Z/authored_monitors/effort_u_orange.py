def check(ctx):
    held = ctx.held_object()
    if held != "u_orange":
        return None
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = j["effort"]
    if e < 0:
        e = -e
    if e > ctx.value("max_effort"):
        return {"value": e, "detail": "finger_joint effort exceeded u_orange's tolerated clamp effort while held"}
    return None