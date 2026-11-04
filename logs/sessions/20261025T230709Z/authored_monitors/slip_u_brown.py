PART = "u_brown"

def check(ctx):
    if ctx.held_object() != PART:
        ctx.state["released"] = True
        return None
    if ctx.state.get("released"):
        return None
    t = ctx.transform("tcp", PART)["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = t
        ctx.state["max_dev"] = 0.0
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.state["max_dev"]:
        ctx.state["max_dev"] = d
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": PART + " moved relative to tcp by " + str(round(d, 4)) + " m while held"}
    return None
