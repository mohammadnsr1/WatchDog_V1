def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    rel = ctx.transform("tcp", "u_brown")
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = rel["translation"]
        return None
    d = ctx.dist(rel["translation"], ctx.state["baseline"])
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "u_brown moved " + str(d) + "m relative to gripper while held"}
    return None