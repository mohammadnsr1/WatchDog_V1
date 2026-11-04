def check(ctx):
    pose = ctx.object_pose("u_green")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    base = ctx.state["baseline"]
    d = ctx.dist(pos, base)
    tol = ctx.value("disturbance_tolerance_m")
    if d > tol:
        return {"value": d, "detail": "u_green moved from baseline beyond tolerance"}
    return None
