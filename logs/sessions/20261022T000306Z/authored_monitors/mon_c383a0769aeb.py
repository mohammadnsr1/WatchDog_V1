def check(ctx):
    pose = ctx.object_pose("u_brown")
    if pose is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["baseline"], pose["pose"]["position"])
    thresh = ctx.value("disturbance_tolerance_m")
    if d > thresh:
        return {"value": d, "detail": "previously-seated u_brown was disturbed"}
    return None
