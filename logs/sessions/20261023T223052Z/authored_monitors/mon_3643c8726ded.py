def check(ctx):
    tol = ctx.value("tol")
    if "init" not in ctx.state:
        p = ctx.object_pose("base1")
        if p is not None:
            ctx.state["init"] = p["pose"]["position"]
        return None
    p = ctx.object_pose("base1")
    if p is None:
        return None
    d = ctx.dist(p["pose"]["position"], ctx.state["init"])
    if d > tol:
        return {"value": d, "detail": "base1 moved beyond tolerance"}
    return None
