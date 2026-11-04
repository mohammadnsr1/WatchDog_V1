def check(ctx):
    p = ctx.object_pose("line_green")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "line_green moved while inactive (not yet being worked on)"}
    return None
