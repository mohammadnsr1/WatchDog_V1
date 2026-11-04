def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    rel = ctx.transform("tcp", "line_green")
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = rel["translation"]
        return None
    d = ctx.dist(rel["translation"], ctx.state["baseline"])
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "line_green moved " + str(d) + "m relative to gripper while held"}
    return None