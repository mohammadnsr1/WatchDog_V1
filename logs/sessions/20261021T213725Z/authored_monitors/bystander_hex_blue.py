def check(ctx):
    pose = ctx.object_pose("hex_blue")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "orig" not in ctx.state:
        ctx.state["orig"] = pos
        return None
    d = ctx.dist(ctx.state["orig"], pos)
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "hex_blue moved " + str(d) + "m from its rest pose"}
    return None
