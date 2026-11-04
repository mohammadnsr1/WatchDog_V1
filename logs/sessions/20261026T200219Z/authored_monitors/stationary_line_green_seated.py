OBJ = "line_green"

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    p = ctx.object_pose(OBJ)
    if "ref" not in ctx.state:
        if p is not None:
            ctx.state["ref"] = p["pose"]["position"]
            ctx.state["ref_wall_ns"] = p["wall_ns"]
        return None
    if p is None:
        return None
    d = ctx.dist(p["pose"]["position"], ctx.state["ref"])
    if d > tol:
        return {"value": d, "detail": OBJ + " moved " + str(round(d, 4)) + " m from its seated rest pose after its place step closed"}
    return None
