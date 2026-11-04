def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    eff = abs(j["effort"])
    thr = ctx.value("max_clamp_effort")
    if eff > thr:
        return {"value": eff, "detail": "finger_joint effort " + str(eff) + " exceeds max_clamp_effort"}
    return None
