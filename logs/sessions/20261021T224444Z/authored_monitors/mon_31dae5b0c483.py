def check(ctx):
    if ctx.held_object() != "inverted_u_yellow":
        return None
    tcp = ctx.frame_pose("tcp")
    pose = ctx.object_pose("inverted_u_yellow")
    if tcp is None or pose is None:
        return None
    dq = ctx.dist(tcp["pose"]["position"], pose["pose"]["position"])
    if "init" not in ctx.state:
        ctx.state["init"] = dq
        return None
    tol = ctx.value("grasp_slip_tolerance_m")
    delta = abs(dq - ctx.state["init"])
    if delta > tol:
        return {"value": delta, "detail": "inverted_u_yellow distance-to-tcp changed by " + str(delta) + "m, suggesting slip"}
    return None
