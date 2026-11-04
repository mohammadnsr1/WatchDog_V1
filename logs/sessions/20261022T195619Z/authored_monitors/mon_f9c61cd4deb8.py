def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    rel = ctx.transform("tcp", "line_green")
    t = rel["translation"]
    if "initial" not in ctx.state:
        ctx.state["initial"] = t
        return None
    d = ctx.dist(ctx.state["initial"], t)
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "line_green slipped " + str(d) + "m relative to gripper"}
    return None
