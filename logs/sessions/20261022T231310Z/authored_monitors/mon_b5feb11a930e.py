def check(ctx):
    pose = ctx.object_pose("hex_blue")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    tol = ctx.value("tol")
    if d > tol:
        return {"value": d, "detail": "hex_blue moved from its rest pose"}
    return None
