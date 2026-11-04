def check(ctx):
    obj = "u_brown"
    p = ctx.object_pose(obj)
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": obj + " moved while inactive, displaced " + str(d) + " m"}
    return None