def check(ctx):
    pose = ctx.object_pose("hex_red")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "init" not in ctx.state:
        ctx.state["init"] = pos
        return None
    d = ctx.dist(pos, ctx.state["init"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "hex_red moved " + str(d) + "m from initial pose"}
    return None
