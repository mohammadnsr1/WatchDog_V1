def check(ctx):
    p = ctx.object_pose("u_green")
    if p is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = p["pose"]["position"]
        return None
    d = ctx.dist(p["pose"]["position"], ctx.state["baseline"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "u_green moved " + str(d) + "m from run-start pose"}
    return None