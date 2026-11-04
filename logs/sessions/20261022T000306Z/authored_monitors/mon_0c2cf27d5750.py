def check(ctx):
    if ctx.held_object() != "line_green":
        return None
    op = ctx.object_pose("line_green")
    tp = ctx.frame_pose("tcp")
    if op is None or tp is None:
        return None
    rel = {"x": op["pose"]["position"]["x"] - tp["pose"]["position"]["x"],
           "y": op["pose"]["position"]["y"] - tp["pose"]["position"]["y"],
           "z": op["pose"]["position"]["z"] - tp["pose"]["position"]["z"]}
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = rel
        return None
    b = ctx.state["baseline"]
    d = ((rel["x"]-b["x"])**2 + (rel["y"]-b["y"])**2 + (rel["z"]-b["z"])**2) ** 0.5
    thresh = ctx.value("grasp_slip_tolerance_m")
    if d > thresh:
        return {"value": d, "detail": "line_green shifted relative to the gripper while held"}
    return None
