def check(ctx):
    oid = "u_orange"
    pose = ctx.object_pose(oid)
    if pose is None:
        return None
    if "origin" not in ctx.state:
        ctx.state["origin"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["origin"], pose["pose"]["position"])
    thr = ctx.value("position_m")
    if d > thr:
        return {"value": d, "detail": "seated u_orange moved " + str(d) + "m from its placed position"}
    return None
