OBJECT_NAME = "u_orange"
def check(ctx):
    obj = ctx.object_pose(OBJECT_NAME)
    if obj is None:
        return None
    pos = obj["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(ctx.state["ref"], pos)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "u_orange moved while inactive (not yet being worked on)"}
    return None
