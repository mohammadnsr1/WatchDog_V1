def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = j["effort"]
    if e > ctx.value("limit"):
        return {"value": e, "detail": "finger_joint effort exceeded line_green's tolerated clamp effort"}
    return None
