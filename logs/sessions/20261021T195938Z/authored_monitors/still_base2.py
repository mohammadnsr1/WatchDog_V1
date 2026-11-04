def check(ctx):
    pose = ctx.object_pose("base2")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": "base2 moved " + str(d) + "m from rest pose, bystander disturbance"}
    return None
