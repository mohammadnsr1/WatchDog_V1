def check(ctx):
    pose = ctx.object_pose("base1")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "orig" not in ctx.state:
        ctx.state["orig"] = pos
        return None
    d = ctx.dist(ctx.state["orig"], pos)
    thr = ctx.value("watch_threshold_m")
    if d > thr:
        return {"value": d, "detail": "base1 (fixture) moved " + str(d) + "m from its rest pose"}
    return None
