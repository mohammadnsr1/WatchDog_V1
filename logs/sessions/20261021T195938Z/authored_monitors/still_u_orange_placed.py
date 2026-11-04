def check(ctx):
    pose = ctx.object_pose("u_orange")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": "u_orange moved " + str(d) + "m after being placed/seated"}
    return None
