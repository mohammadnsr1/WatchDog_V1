def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    tr = ctx.transform("tcp", "line_green")
    pos = tr["translation"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    base = ctx.state["baseline"]
    d = ctx.dist(pos, base)
    tol = ctx.value("tol")
    if d > tol:
        return {"value": d, "detail": "line_green slipped relative to gripper"}
    return None
