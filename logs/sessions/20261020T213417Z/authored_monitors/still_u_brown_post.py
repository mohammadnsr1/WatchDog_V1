OBJ = "u_brown"
def check(ctx):
    p = ctx.object_pose(OBJ)
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("tol_m"):
        return {"value": d, "detail": OBJ + " moved from its seated placement pose"}
    return None
