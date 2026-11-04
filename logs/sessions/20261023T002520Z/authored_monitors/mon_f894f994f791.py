def check(ctx):
    pose = ctx.object_pose("inverted_u_brown")
    if pose is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["baseline"], pose["pose"]["position"])
    tol = ctx.value("disturbance_tolerance_m")
    if d > tol:
        return {"value": d, "detail": "inverted_u_brown moved " + str(d) + "m from baseline, tol " + str(tol)}
    return None
