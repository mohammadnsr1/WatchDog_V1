def check(ctx):
    pose = ctx.object_pose("u_orange")
    if pose is None:
        return None
    if "seated" not in ctx.state:
        ctx.state["seated"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["seated"], pose["pose"]["position"])
    thr = ctx.value("position_m")
    if d > thr:
        return {"value": d, "detail": "seated u_orange moved " + str(d) + "m from its seated pose"}
    return None
