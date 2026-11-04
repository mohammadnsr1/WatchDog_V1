def check(ctx):
    held = ctx.held_object()
    if held != "u_orange":
        return None
    obj = ctx.object_pose("u_orange")
    tcp = ctx.frame_pose("tcp")
    if obj is None or tcp is None:
        return None
    d = ctx.dist(obj["pose"]["position"], tcp["pose"]["position"])
    if "ref" not in ctx.state:
        ctx.state["ref"] = d
        return None
    drift = abs(d - ctx.state["ref"])
    if drift > ctx.value("slip_m"):
        return {"value": drift, "detail": "u_orange drifted " + str(drift) + "m relative to gripper TCP while held"}
    return None
