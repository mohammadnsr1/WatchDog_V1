def check(ctx):
    p = ctx.object_pose("base2")
    if p is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = p["pose"]["position"]
        return None
    d = ctx.dist(p["pose"]["position"], ctx.state["baseline"])
    thr = ctx.value("trigger_m")
    if d > thr:
        return {"value": d, "detail": "base2 appears to have moved " + str(d) + "m from run-start pose"}
    return None