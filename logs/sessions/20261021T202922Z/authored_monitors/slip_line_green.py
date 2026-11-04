def check(ctx):
    held = ctx.held_object()
    if held != "line_green":
        return None
    obj = ctx.object_pose("line_green")
    tcp = ctx.frame_pose("tcp")
    if obj is None or tcp is None:
        return None
    op = obj["pose"]["position"]
    tp = tcp["pose"]["position"]
    dx = op["x"] - tp["x"]
    dy = op["y"] - tp["y"]
    dz = op["z"] - tp["z"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": dx, "y": dy, "z": dz}
        return None
    ref = ctx.state["ref"]
    ddx = dx - ref["x"]
    ddy = dy - ref["y"]
    ddz = dz - ref["z"]
    drift = (ddx*ddx + ddy*ddy + ddz*ddz) ** 0.5
    if drift > ctx.value("slip_tol"):
        return {"value": drift, "detail": "line_green shifted relative to gripper TCP by " + str(drift) + " m while held"}
    return None
