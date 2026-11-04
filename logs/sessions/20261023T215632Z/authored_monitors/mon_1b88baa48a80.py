def check(ctx):
    if "pos" not in ctx.state:
        p = ctx.object_pose("base2")
        if p is None:
            return None
        ctx.state["pos"] = p["pose"]["position"]
        return None
    p = ctx.object_pose("base2")
    if p is None:
        return None
    d = ctx.dist(ctx.state["pos"], p["pose"]["position"])
    thr = ctx.value("watch_threshold_m")
    if d > thr:
        return {"value": d, "detail": "base2 moved " + str(d) + "m from baseline"}
    return None
