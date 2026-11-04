def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    thr = ctx.value("max_clamp_effort")
    if abs(j["effort"]) > thr:
        return {"value": abs(j["effort"]), "detail": "finger_joint effort exceeded max_clamp_effort while holding u_orange"}
    return None
