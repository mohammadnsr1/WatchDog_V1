OBJ = "u_green"
def check(ctx):
    pose = ctx.object_pose(OBJ)
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(p, ref)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": OBJ + " moved " + str(d) + " m from its resting pose"}
    return None
