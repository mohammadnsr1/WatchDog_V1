OBJECT_NAME = "inverted_u_brown"
def check(ctx):
    obj = ctx.object_pose(OBJECT_NAME)
    if obj is None:
        return None
    pos = obj["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(ctx.state["ref"], pos)
    if d > ctx.value("move_tol_m"):
        return {"value": d, "detail": "inverted_u_brown displaced from its reference resting position"}
    return None
