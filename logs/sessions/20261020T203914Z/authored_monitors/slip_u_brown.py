def check(ctx):
    held = ctx.held_object()
    obj = ctx.object_pose("u_brown")
    tcp = ctx.frame_pose("tcp")
    if obj is None or tcp is None:
        return None
    op = obj["pose"]["position"]
    tp = tcp["pose"]["position"]
    rel = {"x": op["x"]-tp["x"], "y": op["y"]-tp["y"], "z": op["z"]-tp["z"]}
    if "ref" not in ctx.state:
        if held == "u_brown":
            ctx.state["ref"] = rel
        return None
    ref = ctx.state["ref"]
    d = ((rel["x"]-ref["x"])**2 + (rel["y"]-ref["y"])**2 + (rel["z"]-ref["z"])**2) ** 0.5
    if d > ctx.value("slip_tol"):
        return {"value": d, "detail": "u_brown shifted " + str(d) + "m relative to gripper while held"}
    return None
