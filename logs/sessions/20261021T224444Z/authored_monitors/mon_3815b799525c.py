def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    tcp = ctx.frame_pose("tcp")
    pose = ctx.object_pose("line_green")
    if tcp is None or pose is None:
        return None
    dq = ctx.dist(tcp["pose"]["position"], pose["pose"]["position"])
    if "init" not in ctx.state:
        ctx.state["init"] = dq
        return None
    tol = ctx.value("grasp_slip_tolerance_m")
    delta = abs(dq - ctx.state["init"])
    if delta > tol:
        return {"value": delta, "detail": "line_green distance-to-tcp changed by " + str(delta) + "m, suggesting slip"}
    return None
