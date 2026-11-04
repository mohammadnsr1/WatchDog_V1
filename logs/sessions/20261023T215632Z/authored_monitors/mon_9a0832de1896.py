def check(ctx):
    held = ctx.held_object()
    if held != "inverted_u_yellow":
        return None
    tcp = ctx.frame_pose("tcp")
    obj = ctx.object_pose("inverted_u_yellow")
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
        return {"value": d, "detail": "inverted_u_yellow shifted " + str(d) + "m relative to gripper while held"}
    return None
