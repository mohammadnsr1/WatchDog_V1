OBJS = ["u_orange", "line_green", "inverted_u_yellow"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    ref = ctx.state.get("ref")
    if ref is None:
        ref = {}
        for o in OBJS:
            p = ctx.object_pose(o)
            if p is not None:
                ref[o] = p["pose"]["position"]
        ctx.state["ref"] = ref
        return None
    worst = 0.0
    worst_o = None
    for o in OBJS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        if o not in ref:
            ref[o] = p["pose"]["position"]
            continue
        d = ctx.dist(p["pose"]["position"], ref[o])
        if d > worst:
            worst = d
            worst_o = o
    if worst > tol:
        return {"value": worst, "detail": str(worst_o) + " displaced " + str(round(worst, 4)) + " m from its last rest pose"}
    return None
