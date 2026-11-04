def check(ctx):
    j = ctx.joint("finger_joint")
    lim = ctx.value("max_clamp_effort")
    if "peak" not in ctx.state:
        ctx.state["peak"] = 0.0
        ctx.state["first_effort"] = None if j is None else j["effort"]
        return None
    if j is None:
        return None
    e = abs(j["effort"])
    if e > ctx.state["peak"]:
        ctx.state["peak"] = e
    if e > lim:
        return {"value": float(e), "detail": "finger_joint |effort| %.3f exceeds line_green max_clamp_effort %.1f (held=%s)" % (e, lim, str(ctx.held_object()))}
    return None
