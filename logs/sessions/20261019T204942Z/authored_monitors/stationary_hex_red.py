OBJECT_ID = "hex_red"
def check(ctx):
    pose = ctx.object_pose(OBJECT_ID)
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "hex_red moved from rest, dist=" + str(d)}
    return None
