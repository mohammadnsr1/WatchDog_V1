def check(ctx):
    pose = ctx.object_pose("u_orange")
    if pose is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], pose["pose"]["position"])
    tol = ctx.value("position_m")
    if d > tol:
        return {"value": d, "detail": "seated u_orange moved beyond its registry position tolerance"}
    return None
