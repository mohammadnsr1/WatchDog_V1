def check(ctx):
    p = ctx.object_pose("hex_blue")
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    tol = ctx.value("disturbance_tolerance_m")
    if d > tol:
        return {"value": d, "detail": "hex_blue moved beyond disturbance tolerance"}
    return None