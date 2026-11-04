def check(ctx):
    p = ctx.object_pose("u_orange")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "u_orange moved from its seated pose by " + str(d) + " m after place_u_orange closed"}
    return None
