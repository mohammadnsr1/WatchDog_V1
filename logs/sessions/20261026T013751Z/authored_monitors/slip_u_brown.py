def check(ctx):
    held = ctx.held_object()
    t = ctx.transform("tcp", "u_brown")["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": t["x"], "y": t["y"], "z": t["z"]}
        ctx.state["ref_dist"] = math.sqrt(t["x"] ** 2 + t["y"] ** 2 + t["z"] ** 2)
        ctx.state["held_at_arm"] = held
        return None
    if held != "u_brown":
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(d), "detail": "u_brown moved " + str(round(d, 4)) + " m relative to tcp while held"}
    return None
