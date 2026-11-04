OBJ = "inverted_u_yellow"

def check(ctx):
    p = ctx.object_pose(OBJ)
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        ctx.state["ref_wall_ns"] = p["wall_ns"]
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": OBJ + " displaced %.4f m from rest pose captured at arm time" % d}
    return None
