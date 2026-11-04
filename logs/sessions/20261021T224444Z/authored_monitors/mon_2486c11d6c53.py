def check(ctx):
    obj = "hex_blue"
    pose = ctx.object_pose(obj)
    if pose is None:
        return None
    if "init" not in ctx.state:
        ctx.state["init"] = pose["pose"]["position"]
        return None
    tol = ctx.value("disturbance_tolerance_m")
    d = ctx.dist(ctx.state["init"], pose["pose"]["position"])
    if d > tol:
        return {"value": d, "detail": "hex_blue moved " + str(d) + "m from initial rest"}
    return None
