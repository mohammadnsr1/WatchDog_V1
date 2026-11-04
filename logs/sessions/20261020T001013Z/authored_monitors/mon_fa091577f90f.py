OBJ = "inverted_u_yellow"
def check(ctx):
    pose = ctx.object_pose(OBJ)
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    b = ctx.state["baseline"]
    d = ctx.dist(pos, b)
    if d > ctx.value("tol"):
        return {"value": d, "detail": OBJ + " moved " + str(d) + " m from baseline while inactive"}
    return None
