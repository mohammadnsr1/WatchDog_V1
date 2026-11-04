def check(ctx):
    pose = ctx.object_pose("hex_blue")
    if pose is None:
        return None
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pose["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["baseline"], pose["pose"]["position"])
    thresh = ctx.value("disturbance_tolerance_m")
    if d > thresh:
        return {"value": d, "detail": "hex_blue moved from its run-start baseline"}
    return None
