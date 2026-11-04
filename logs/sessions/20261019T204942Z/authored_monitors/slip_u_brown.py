OBJECT_ID = "u_brown"
def check(ctx):
    held = ctx.held_object()
    if held != OBJECT_ID:
        return None
    obj = ctx.object_pose(OBJECT_ID)
    tcp = ctx.frame_pose("tcp")
    if obj is None or tcp is None:
        return None
    d = ctx.dist(obj["pose"]["position"], tcp["pose"]["position"])
    if "ref" not in ctx.state:
        ctx.state["ref"] = d
        return None
    drift = abs(d - ctx.state["ref"])
    if drift > ctx.value("grasp_slip_tolerance_m"):
        return {"value": drift, "detail": "u_brown moved relative to gripper TCP by " + str(drift) + "m while held"}
    return None
