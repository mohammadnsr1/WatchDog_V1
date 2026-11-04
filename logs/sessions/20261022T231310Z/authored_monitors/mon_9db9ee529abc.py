def check(ctx):
    pose = ctx.object_pose("base1")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    tol = ctx.value("tol")
    if d > tol:
        return {"value": d, "detail": "base1 (fixture) moved from its rest pose"}
    return None
