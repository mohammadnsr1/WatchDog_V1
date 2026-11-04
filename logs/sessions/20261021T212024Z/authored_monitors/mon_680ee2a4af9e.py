def check(ctx):
    pose = ctx.object_pose("line_green")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    base = ctx.state["baseline"]
    d = ctx.dist(pos, base)
    tol = ctx.value("tol")
    if d > tol:
        return {"value": d, "detail": "previously seated line_green moved beyond position tolerance"}
    return None
