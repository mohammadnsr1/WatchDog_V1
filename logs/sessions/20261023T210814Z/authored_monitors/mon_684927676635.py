def check(ctx):
    p = ctx.object_pose("hex_blue")
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "hex_blue moved from reference pose"}
    return None
