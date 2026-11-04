OID = "u_brown"

def check(ctx):
    if ctx.held_object() != OID:
        return None
    t = ctx.transform("tcp", OID)["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = t
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": OID + " moved " + str(round(d, 4)) + " m relative to tcp while held (slip or lost grasp)"}
    return None
