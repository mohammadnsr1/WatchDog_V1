def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    t = ctx.transform("tcp", "line_green")
    if "ref" not in ctx.state:
        ctx.state["ref"] = t["translation"]
        return None
    d = ctx.dist(ctx.state["ref"], t["translation"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": "line_green moved relative to tcp (slip)"}
    return None
