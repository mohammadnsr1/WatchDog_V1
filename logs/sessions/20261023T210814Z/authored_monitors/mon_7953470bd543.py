def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    t = ctx.transform("tcp", "inverted_u_yellow")
    if "ref" not in ctx.state:
        ctx.state["ref"] = t["translation"]
        return None
    d = ctx.dist(ctx.state["ref"], t["translation"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": "inverted_u_yellow moved relative to tcp (slip)"}
    return None
