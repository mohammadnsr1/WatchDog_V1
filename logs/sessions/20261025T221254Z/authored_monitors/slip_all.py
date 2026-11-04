def check(ctx):
    tol = ctx.value("grasp_slip_tolerance_m")
    h = ctx.held_object()
    now = ctx.now_wall_ns()
    if h is None:
        ctx.state.pop("held", None)
        ctx.state.pop("ref", None)
        return None
    if ctx.state.get("held") != h:
        ctx.state["held"] = h
        ctx.state["t0"] = now
        ctx.state.pop("ref", None)
        return None
    o = ctx.object_pose(h)
    t = ctx.frame_pose("tcp")
    if o is None or t is None:
        return None
    a = o["pose"]["position"]; b = t["pose"]["position"]
    off = {"x": a["x"] - b["x"], "y": a["y"] - b["y"], "z": a["z"] - b["z"]}
    if "ref" not in ctx.state:
        if now - ctx.state["t0"] < 3000000000:
            return None
        ctx.state["ref"] = off
        return None
    d = ctx.dist(off, ctx.state["ref"])
    if d > tol:
        return {"value": d, "detail": h + " moved " + str(round(d, 4)) + " m relative to tcp while held"}
    return None
