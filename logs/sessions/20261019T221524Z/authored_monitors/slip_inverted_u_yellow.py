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
    slip = abs(d - ctx.state["ref"])
    tol = ctx.value("grasp_slip_tolerance_m")
    if slip > tol:
        return {"value": slip, "detail": "inverted_u_yellow-to-tcp offset changed by " + str(slip) + " m while held, exceeding slip tolerance " + str(tol)}
    return None