WATCH = ["u_green", "base2", "hex_blue", "hex_red", "inverted_u_brown", "base1"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for oid in WATCH:
            p = ctx.object_pose(oid)
            if p is not None:
                ref[oid] = p["pose"]["position"]
        ctx.state["ref"] = ref
        ctx.state["max_disp"] = {}
        return None
    ref = ctx.state["ref"]
    worst = None
    for oid in WATCH:
        p = ctx.object_pose(oid)
        if p is None:
            continue
        if oid not in ref:
            ref[oid] = p["pose"]["position"]
            continue
        d = ctx.dist(p["pose"]["position"], ref[oid])
        if d > ctx.state["max_disp"].get(oid, 0.0):
            ctx.state["max_disp"][oid] = d
        if d > tol and (worst is None or d > worst[1]):
            worst = (oid, d)
    if worst is not None:
        return {"value": float(worst[1]), "detail": "%s displaced %.4f m from its run-start rest pose (tol %.3f)" % (worst[0], worst[1], tol)}
    return None
