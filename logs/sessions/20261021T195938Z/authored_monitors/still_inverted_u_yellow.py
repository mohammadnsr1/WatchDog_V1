def check(ctx):
    pose = ctx.object_pose("inverted_u_yellow")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": "inverted_u_yellow moved " + str(d) + "m while inactive (not yet being worked on)"}
    return None
