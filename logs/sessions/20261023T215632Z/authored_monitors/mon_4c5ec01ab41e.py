def check(ctx):
    held = ctx.held_object()
    if held != "line_green":
        return None
    tcp = ctx.frame_pose("tcp")
    obj = ctx.object_pose("line_green")
    if tcp is None or obj is None:
        return None
    rel = {"x": obj["pose"]["position"]["x"] - tcp["pose"]["position"]["x"],
           "y": obj["pose"]["position"]["y"] - tcp["pose"]["position"]["y"],
           "z": obj["pose"]["position"]["z"] - tcp["pose"]["position"]["z"]}
    if "rel0" not in ctx.state:
        ctx.state["rel0"] = rel
        return None
    d = ctx.dist(ctx.state["rel0"], rel)
    thr = ctx.value("grasp_slip_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "line_green shifted " + str(d) + "m relative to gripper while held"}
    return None
