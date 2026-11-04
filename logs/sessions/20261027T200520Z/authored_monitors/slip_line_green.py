OBJ = "line_green"

def check(ctx):
    if ctx.held_object() != OBJ:
        return None
    t = ctx.transform("tcp", OBJ)["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": t["x"], "y": t["y"], "z": t["z"]}
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": OBJ + " moved %.4f m relative to tcp while held" % d}
    return None
