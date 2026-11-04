OBJS = ["base1", "base2", "u_green", "inverted_u_brown", "hex_blue", "hex_red"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for o in OBJS:
            p = ctx.object_pose(o)
            if p is not None:
                ref[o] = p["pose"]["position"]
        ctx.state["ref"] = ref
        ctx.state["ref_wall_ns"] = ctx.now_wall_ns()
        return None
    ref = ctx.state["ref"]
    worst = 0.0
    wo = None
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
            wo = o
    if worst > tol:
        return {"value": worst, "detail": str(wo) + " displaced " + str(round(worst, 4)) + " m from its reference rest pose"}
    return None
