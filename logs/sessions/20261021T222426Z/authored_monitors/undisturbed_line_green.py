def check(ctx):
    pose = ctx.object_pose("line_green")
    if pose is None:
        return None
    if "p0" not in ctx.state:
        ctx.state["p0"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["p0"], pose["pose"]["position"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "seated line_green moved " + str(d) + "m from its placed position"}
    return None