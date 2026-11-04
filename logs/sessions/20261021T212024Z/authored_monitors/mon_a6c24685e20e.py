def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    tr = ctx.transform("tcp", "inverted_u_yellow")
    pos = tr["translation"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    base = ctx.state["baseline"]
    d = ctx.dist(pos, base)
    tol = ctx.value("tol")
    if d > tol:
        return {"value": d, "detail": "inverted_u_yellow slipped relative to gripper"}
    return None
