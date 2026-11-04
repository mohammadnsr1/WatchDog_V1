def check(ctx):
    pose = ctx.object_pose("inverted_u_brown")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_m"):
        return {"value": d, "detail": "inverted_u_brown moved from its last rest pose"}
    return None
