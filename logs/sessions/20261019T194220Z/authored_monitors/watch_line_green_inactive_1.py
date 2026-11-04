OBJECT_NAME = "line_green"
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
        return {"value": d, "detail": "line_green moved while inactive (not yet being worked on)"}
    return None
