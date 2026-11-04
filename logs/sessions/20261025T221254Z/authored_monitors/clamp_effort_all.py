def check(ctx):
    lim = {"u_brown": ctx.value("max_u_brown"), "u_orange": ctx.value("max_u_orange"),
           "line_green": ctx.value("max_line_green"), "inverted_u_yellow": ctx.value("max_inverted_u_yellow")}
    j = ctx.joint("finger_joint")
    ctx.state["seen"] = j is not None
    h = ctx.held_object()
    if j is None or h is None or h not in lim:
        return None
    e = abs(j["effort"])
    if e > lim[h]:
        return {"value": e, "detail": "finger_joint effort " + str(round(e, 2)) + " > " + str(lim[h]) + " for " + h}
    return None
