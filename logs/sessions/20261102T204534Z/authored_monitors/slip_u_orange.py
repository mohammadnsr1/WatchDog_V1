OBJ = "u_orange"

def check(ctx):
    held = ctx.held_object()
    if "ref" not in ctx.state:
        t = ctx.transform("tcp", OBJ)["translation"]
        ctx.state["ref"] = t
        ctx.state["held_at_arm"] = held
        ctx.state["max_slip"] = 0.0
        ctx.state["released"] = False
        return None
    if ctx.state["released"]:
        return None
    if held != OBJ:
        ctx.state["released"] = True
        return None
    t = ctx.transform("tcp", OBJ)["translation"]
    s = ctx.dist(t, ctx.state["ref"])
    if s > ctx.state["max_slip"]:
        ctx.state["max_slip"] = s
    if s > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(s), "detail": "u_orange moved %.4f m relative to tcp while held" % s}
    return None
