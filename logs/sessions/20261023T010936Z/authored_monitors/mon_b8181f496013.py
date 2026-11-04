def check(ctx):
    p = ctx.object_pose("hex_blue")
    if p is None:
        return None
    if "pos" not in ctx.state:
        ctx.state["pos"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["pos"], p["pose"]["position"])
    thr = ctx.value("disturbance_tolerance_m")
    if d > thr:
        return {"value": d, "detail": "hex_blue moved " + str(d) + "m from rest pose"}
    return None
