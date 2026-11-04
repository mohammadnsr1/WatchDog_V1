OBJECT_ID = "base2"

def check(ctx):
    pose = ctx.object_pose(OBJECT_ID)
    if pose is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pose["pose"]["position"]
        return None
    d = ctx.dist(pose["pose"]["position"], ctx.state["baseline"])
    thr = ctx.value("agent_chosen_threshold_m")
    if d > thr:
        return {"value": d, "detail": "base2 moved " + str(d) + "m from baseline"}
    return None
