def check(ctx):
    p = ctx.object_pose("hex_red")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "hex_red moved from rest pose by " + str(d) + " m"}
    return None
