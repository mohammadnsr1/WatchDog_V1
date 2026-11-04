WATCH = ["u_brown", "u_orange", "line_green"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for oid in WATCH:
            p = ctx.object_pose(oid)
            if p is not None:
                ref[oid] = p["pose"]["position"]
        ctx.state["ref"] = ref
        return None
    ref = ctx.state["ref"]
    for oid in WATCH:
        p = ctx.object_pose(oid)
        if p is None:
            continue
        if oid not in ref:
            ref[oid] = p["pose"]["position"]
            continue
        d = ctx.dist(p["pose"]["position"], ref[oid])
        if d > tol:
            return {"value": float(d), "detail": "%s (inactive during inverted_u_yellow span) displaced %.4f m from where it last came to rest" % (oid, d)}
    return None
