def check(ctx):
    p = ctx.object_pose("base1")
    if p is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = p["pose"]["position"]
        return None
    d = ctx.dist(ctx.state["ref"], p["pose"]["position"])
    if d > ctx.value("trigger_m"):
        return {"value": d, "detail": "base1 moved from reference pose"}
    return None
