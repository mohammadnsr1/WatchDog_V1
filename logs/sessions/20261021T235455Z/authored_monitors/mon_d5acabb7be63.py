def check(ctx):
    p = ctx.object_pose("inverted_u_brown")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    b = ctx.state["baseline"]
    d = ctx.dist(pos, b)
    tol = ctx.value("disturbance_tolerance_m")
    if d > tol:
        return {"value": d, "detail": "inverted_u_brown moved " + str(d) + "m from baseline"}
    return None
