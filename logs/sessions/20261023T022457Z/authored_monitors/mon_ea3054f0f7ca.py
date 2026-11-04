def check(ctx):
    eff = ctx.joint("finger_joint")
    if eff is None:
        return None
    thr = ctx.value("max_clamp_effort")
    e = abs(eff["effort"])
    if e > thr:
        return {"value": e, "detail": "clamp effort on line_green exceeded limit: " + str(e)}
    return None
