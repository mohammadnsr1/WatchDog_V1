def check(ctx):
    pose = ctx.object_pose("base2")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    b = ctx.state["baseline"]
    d = ctx.dist(pos, b)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "base2 moved from its resting pose"}
    return None
