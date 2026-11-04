def check(ctx):
    obj = "u_orange"
    p = ctx.object_pose(obj)
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": obj + " displaced after being seated, by " + str(d) + " m"}
    return None