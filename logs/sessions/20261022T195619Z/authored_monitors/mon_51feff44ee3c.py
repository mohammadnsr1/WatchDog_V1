def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    rel = ctx.transform("tcp", "inverted_u_yellow")
    t = rel["translation"]
    if "initial" not in ctx.state:
        ctx.state["initial"] = t
        return None
    d = ctx.dist(ctx.state["initial"], t)
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "inverted_u_yellow slipped " + str(d) + "m relative to gripper"}
    return None
