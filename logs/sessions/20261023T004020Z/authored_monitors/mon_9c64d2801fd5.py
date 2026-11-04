OBJECT_ID = "u_green"

def check(ctx):
    pose = ctx.object_pose(OBJECT_ID)
    if pose is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pose["pose"]["position"]
        return None
    d = ctx.dist(pose["pose"]["position"], ctx.state["baseline"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "u_green moved " + str(d) + "m from baseline"}
    return None
