def check(ctx):
    held = ctx.held_object()
    t = ctx.transform("tcp", "line_green")["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": t["x"], "y": t["y"], "z": t["z"]}
        ctx.state["held_at_arm"] = held
        return None
    if held != "line_green":
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(d), "detail": "line_green moved " + str(round(d, 4)) + " m relative to tcp while held"}
    return None
