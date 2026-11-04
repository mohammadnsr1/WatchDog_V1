def check(ctx):
    held = ctx.held_object()
    tcp = ctx.frame_pose("tcp")
    obj = ctx.object_pose("inverted_u_yellow")
    if held != "inverted_u_yellow" or tcp is None or obj is None:
        return None
    off = ctx.dist(tcp["pose"]["position"], obj["pose"]["position"])
    if "ref" not in ctx.state:
        ctx.state["ref"] = off
        return None
    drift = abs(off - ctx.state["ref"])
    if drift > ctx.value("slip_tol"):
        return {"value": drift, "detail": "inverted_u_yellow moved relative to gripper (in-hand slip) while held"}
    return None
