def check(ctx):
    pose = ctx.object_pose("hex_blue")
    if pose is None:
        return None
    if "initial" not in ctx.state:
        ctx.state["initial"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["initial"], pose["pose"]["position"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "hex_blue moved " + str(d) + "m from its initial position"}
    return None
