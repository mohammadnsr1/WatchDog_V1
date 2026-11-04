OBJ = "inverted_u_yellow"

def check(ctx):
    lim = ctx.value("max_clamp_effort")
    w = ctx.joint_window("finger_joint", 0.5)
    if "init" not in ctx.state:
        ctx.state["init"] = True
        ctx.state["max_abs_effort"] = 0.0
        j = ctx.joint("finger_joint")
        ctx.state["effort_at_arm"] = None if j is None else j["effort"]
        return None
    peak = 0.0
    for s in w:
        e = s["effort"]
        if e is not None and abs(e) > peak:
            peak = abs(e)
    if peak > ctx.state["max_abs_effort"]:
        ctx.state["max_abs_effort"] = peak
    if peak > lim:
        return {"value": peak, "detail": "finger_joint |effort| " + str(round(peak, 3)) + " exceeded max_clamp_effort for " + OBJ}
    return None
