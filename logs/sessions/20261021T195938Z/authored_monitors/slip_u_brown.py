def check(ctx):
    held = ctx.held_object()
    tcp = ctx.frame_pose("tcp")
    obj = ctx.object_pose("u_brown")
    if held != "u_brown" or tcp is None or obj is None:
        return None
    off = ctx.dist(tcp["pose"]["position"], obj["pose"]["position"])
    if "ref" not in ctx.state:
        ctx.state["ref"] = off
        return None
    drift = abs(off - ctx.state["ref"])
    if drift > ctx.value("slip_tol"):
        return {"value": drift, "detail": "u_brown moved relative to gripper (in-hand slip) while held"}
    return None
