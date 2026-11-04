def check(ctx):
    pose = ctx.object_pose("u_orange")
    if pose is None:
        return None
    if "init" not in ctx.state:
        ctx.state["init"] = pose["pose"]["position"]
        return None
    tol = ctx.value("disturbance_tolerance_m")
    d = ctx.dist(ctx.state["init"], pose["pose"]["position"])
    if d > tol:
        return {"value": d, "detail": "seated u_orange moved " + str(d) + "m after placement"}
    return None
