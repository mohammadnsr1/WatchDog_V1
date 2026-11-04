def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    e = abs(j["effort"])
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort " + str(e) + " exceeded u_brown's tolerated clamp effort"}
    return None
