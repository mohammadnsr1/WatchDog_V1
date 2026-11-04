def check(ctx):
    pose = ctx.object_pose("base1")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": "base1 fixture moved " + str(d) + "m, must never be disturbed"}
    return None
