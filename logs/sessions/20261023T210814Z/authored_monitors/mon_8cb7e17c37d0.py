def check(ctx):
    p = ctx.object_pose("inverted_u_brown")
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "inverted_u_brown moved from reference pose"}
    return None
