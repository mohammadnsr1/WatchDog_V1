def check(ctx):
    pose = ctx.object_pose("base1")
    if pose is None:
        return None
    if "p0" not in ctx.state:
        ctx.state["p0"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["p0"], pose["pose"]["position"])
    thr = ctx.value("trigger_m")
    if d > thr:
        return {"value": d, "detail": "base1 moved " + str(d) + "m from its initial position"}
    return None