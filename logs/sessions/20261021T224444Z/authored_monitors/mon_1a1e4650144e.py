def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    tcp = ctx.frame_pose("tcp")
    pose = ctx.object_pose("u_brown")
    if tcp is None or pose is None:
        return None
    rel = {"x": pose["pose"]["position"]["x"] - tcp["pose"]["position"]["x"],
           "y": pose["pose"]["position"]["y"] - tcp["pose"]["position"]["y"],
           "z": pose["pose"]["position"]["z"] - tcp["pose"]["position"]["z"]}
    if "init" not in ctx.state:
        ctx.state["init"] = rel
        return None
    tol = ctx.value("grasp_slip_tolerance_m")
    d = ctx.dist(ctx.state["init"], rel)
    if d > tol:
        return {"value": d, "detail": "u_brown slipped " + str(d) + "m relative to tcp while held"}
    return None
