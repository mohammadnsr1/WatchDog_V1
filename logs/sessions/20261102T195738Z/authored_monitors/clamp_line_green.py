def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if "seen" not in ctx.state:
        ctx.state["seen"] = e
        return None
    if e > ctx.value("max_clamp_effort"):
        return {"value": float(e), "detail": "finger_joint |effort| %.2f exceeds max_clamp_effort" % e}
    return None
