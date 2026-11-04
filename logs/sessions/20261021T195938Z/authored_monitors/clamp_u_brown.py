def check(ctx):
    e = ctx.joint("finger_joint")
    if e is None:
        return None
    eff = abs(e["effort"])
    if eff > ctx.value("max_eff"):
        return {"value": eff, "detail": "finger_joint instantaneous effort exceeded u_brown's tolerated clamp effort"}
    return None
