def check(ctx):
    pose = ctx.object_pose("line_green")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    b = ctx.state["baseline"]
    d = ctx.dist(pos, b)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "line_green moved after being seated at its placement"}
    return None
