def check(ctx):
    p = ctx.object_pose("base1")
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    tol = ctx.value("watch_threshold_m")
    if d > tol:
        return {"value": d, "detail": "base1 moved from its reference position"}
    return None