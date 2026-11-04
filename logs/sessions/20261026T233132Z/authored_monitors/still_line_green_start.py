OID = "line_green"

def check(ctx):
    p = ctx.object_pose(OID)
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": OID + " moved " + str(round(d, 4)) + " m from its rest pose"}
    return None
