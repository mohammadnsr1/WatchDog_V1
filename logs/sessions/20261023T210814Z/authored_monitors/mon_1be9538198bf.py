def check(ctx):
    if ctx.held_object() != "u_orange":
        return None
    t = ctx.transform("tcp", "u_orange")
    if "ref" not in ctx.state:
        ctx.state["ref"] = t["translation"]
        return None
    d = ctx.dist(ctx.state["ref"], t["translation"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": "u_orange moved relative to tcp (slip)"}
    return None
