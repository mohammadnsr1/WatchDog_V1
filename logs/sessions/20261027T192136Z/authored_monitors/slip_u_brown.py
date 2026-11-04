def rel(ctx):
    t = ctx.transform("tcp", OBJ)["translation"]
    return t

def check(ctx):
    if ctx.held_object() != OBJ:
        return None
    t = rel(ctx)
    if "ref" not in ctx.state:
        ctx.state["ref"] = t
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": OBJ + " moved %.4f m relative to tcp while held (slip / missed grasp)" % d}
    return None

OBJ = "u_brown"
