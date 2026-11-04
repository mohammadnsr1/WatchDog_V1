def check(ctx):
    pose = ctx.object_pose("line_green")
    if pose is None:
        return None
    if "seated" not in ctx.state:
        ctx.state["seated"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["seated"], pose["pose"]["position"])
    thr = ctx.value("position_m")
    if d > thr:
        return {"value": d, "detail": "seated line_green moved " + str(d) + "m from its seated pose"}
    return None
