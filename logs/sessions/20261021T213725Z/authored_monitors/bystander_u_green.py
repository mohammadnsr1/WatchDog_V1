def check(ctx):
    pose = ctx.object_pose("u_green")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "orig" not in ctx.state:
        ctx.state["orig"] = pos
        return None
    d = ctx.dist(ctx.state["orig"], pos)
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "u_green moved " + str(d) + "m from its rest pose"}
    return None
