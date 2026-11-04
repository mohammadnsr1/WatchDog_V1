def check(ctx):
    if ctx.held_object() != "hex_red":
        return None
    t = ctx.transform("tcp", "hex_red")
    if "ref" not in ctx.state:
        ctx.state["ref"] = t["translation"]
        return None
    d = ctx.dist(ctx.state["ref"], t["translation"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": "hex_red moved relative to tcp (slip)"}
    return None
