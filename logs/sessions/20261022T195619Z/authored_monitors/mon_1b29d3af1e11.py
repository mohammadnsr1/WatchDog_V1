def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    rel = ctx.transform("tcp", "u_brown")
    t = rel["translation"]
    if "initial" not in ctx.state:
        ctx.state["initial"] = t
        return None
    d = ctx.dist(ctx.state["initial"], t)
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "u_brown slipped " + str(d) + "m relative to gripper"}
    return None
