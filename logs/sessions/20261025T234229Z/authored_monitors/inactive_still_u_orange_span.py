WATCHED = ["u_brown", "line_green", "inverted_u_yellow"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for oid in WATCHED:
            p = ctx.object_pose(oid)
            if p is not None:
                ref[oid] = p["pose"]["position"]
        ctx.state["ref"] = ref
        return None
    ref = ctx.state["ref"]
    for oid in WATCHED:
        p = ctx.object_pose(oid)
        if p is None:
            continue
        if oid not in ref:
            ref[oid] = p["pose"]["position"]
            continue
        d = ctx.dist(p["pose"]["position"], ref[oid])
        if d > tol:
            return {"value": d, "detail": oid + " displaced " + str(round(d, 4)) + " m from its last rest pose"}
    return None
