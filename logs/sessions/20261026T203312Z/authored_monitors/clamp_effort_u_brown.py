def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = j["effort"]
    st = ctx.state
    if "init_effort" not in st:
        st["init_effort"] = e
        st["limit"] = ctx.value("max_clamp_effort")
        return None
    if abs(e) > ctx.value("max_clamp_effort"):
        return {"value": float(abs(e)), "detail": "finger_joint effort " + str(e) + " exceeds max_clamp_effort"}
    return None
