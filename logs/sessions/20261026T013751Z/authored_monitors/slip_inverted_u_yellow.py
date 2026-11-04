def check(ctx):
    held = ctx.held_object()
    t = ctx.transform("tcp", "inverted_u_yellow")["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": t["x"], "y": t["y"], "z": t["z"]}
        ctx.state["held_at_arm"] = held
        return None
    if held != "inverted_u_yellow":
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(d), "detail": "inverted_u_yellow moved " + str(round(d, 4)) + " m relative to tcp while held"}
    return None
