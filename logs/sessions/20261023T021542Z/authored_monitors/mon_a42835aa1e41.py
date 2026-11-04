def check(ctx):
    pose = ctx.object_pose("base1")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "init" not in ctx.state:
        ctx.state["init"] = pos
        return None
    d = ctx.dist(pos, ctx.state["init"])
    thr = ctx.value("agent_threshold_m")
    if d > thr:
        return {"value": d, "detail": "base1 moved " + str(d) + "m from initial pose"}
    return None
