def check(ctx):
    pose = ctx.object_pose("base2")
    if pose is None:
        return None
    if "initial" not in ctx.state:
        ctx.state["initial"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["initial"], pose["pose"]["position"])
    thr = ctx.value("move_trigger_m")
    if d > thr:
        return {"value": d, "detail": "base2 moved " + str(d) + "m from its initial position"}
    return None
