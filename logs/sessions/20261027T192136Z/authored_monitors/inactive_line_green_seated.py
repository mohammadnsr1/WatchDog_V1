def check(ctx):
    oid = OBJ
    p = ctx.object_pose(oid)
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": oid + " (seated, inactive) moved %.4f m from seated rest pose" % d}
    return None

OBJ = "line_green"
