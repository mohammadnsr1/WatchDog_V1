def check(ctx):
    pose = ctx.object_pose("line_green")
    if pose is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], pose["pose"]["position"])
    tol = ctx.value("position_m")
    if d > tol:
        return {"value": d, "detail": "seated line_green moved beyond its registry position tolerance"}
    return None
