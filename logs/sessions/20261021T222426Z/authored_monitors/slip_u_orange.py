def check(ctx):
    if ctx.held_object() != "u_orange":
        ctx.state["rel0"] = None
        return None
    rel = ctx.transform("tcp", "u_orange")
    if "rel0" not in ctx.state or ctx.state["rel0"] is None:
        ctx.state["rel0"] = rel["translation"]
        return None
    d = ctx.dist(ctx.state["rel0"], rel["translation"])
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "u_orange shifted " + str(d) + "m relative to gripper"}
    return None