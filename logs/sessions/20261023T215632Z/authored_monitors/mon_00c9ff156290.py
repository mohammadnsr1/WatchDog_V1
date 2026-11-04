def check(ctx):
    if "pos" not in ctx.state:
        p = ctx.object_pose("inverted_u_brown")
        if p is None:
            return None
        ctx.state["pos"] = p["pose"]["position"]
        return None
    p = ctx.object_pose("inverted_u_brown")
    if p is None:
        return None
    d = ctx.dist(ctx.state["pos"], p["pose"]["position"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "inverted_u_brown moved " + str(d) + "m from baseline"}
    return None
