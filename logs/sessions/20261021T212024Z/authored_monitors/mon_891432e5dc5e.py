def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    op = ctx.object_pose("u_brown")
    if op is None:
        return None
    tr = ctx.transform("tcp", "u_brown")
    pos = tr["translation"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    base = ctx.state["baseline"]
    d = ctx.dist(pos, base)
    tol = ctx.value("tol")
    if d > tol:
        return {"value": d, "detail": "u_brown slipped relative to gripper"}
    return None
