def check(ctx):
    p = ctx.object_pose("base1")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "base1 fixture moved; must never be disturbed"}
    return None
