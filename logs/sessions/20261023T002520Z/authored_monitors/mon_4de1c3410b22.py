def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None or j.get("effort") is None:
        return None
    effort = abs(j["effort"])
    limit = ctx.value("max_clamp_effort")
    if effort > limit:
        return {"value": effort, "detail": "finger_joint effort " + str(effort) + " exceeds max_clamp_effort " + str(limit)}
    return None
