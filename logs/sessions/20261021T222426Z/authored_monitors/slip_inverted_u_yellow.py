def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        ctx.state["rel0"] = None
        return None
    rel = ctx.transform("tcp", "inverted_u_yellow")
    if "rel0" not in ctx.state or ctx.state["rel0"] is None:
        ctx.state["rel0"] = rel["translation"]
        return None
    d = ctx.dist(ctx.state["rel0"], rel["translation"])
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "inverted_u_yellow shifted " + str(d) + "m relative to gripper"}
    return None