def check(ctx):
    p = ctx.object_pose("base2")
    if p is None:
        return None
    if "pos" not in ctx.state:
        ctx.state["pos"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["pos"], p["pose"]["position"])
    thr = ctx.value("watch_threshold_m")
    if d > thr:
        return {"value": d, "detail": "base2 moved " + str(d) + "m from rest pose"}
    return None
