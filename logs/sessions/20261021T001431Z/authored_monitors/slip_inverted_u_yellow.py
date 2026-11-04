def check(ctx):
    held = ctx.held_object()
    if held != "inverted_u_yellow":
        return None
    obj = ctx.object_pose("inverted_u_yellow")
    tcp = ctx.frame_pose("tcp")
    if obj is None or tcp is None:
        return None
    d = ctx.dist(obj["pose"]["position"], tcp["pose"]["position"])
    if "ref" not in ctx.state:
        ctx.state["ref"] = d
        return None
    drift = abs(d - ctx.state["ref"])
    if drift > ctx.value("slip_m"):
        return {"value": drift, "detail": "inverted_u_yellow drifted " + str(drift) + "m relative to gripper TCP while held"}
    return None
