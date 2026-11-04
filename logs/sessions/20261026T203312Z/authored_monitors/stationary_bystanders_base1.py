OBJS = ["u_green", "inverted_u_brown", "hex_blue", "hex_red", "base2", "base1"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    st = ctx.state
    if "ref" not in st:
        ref = {}
        for o in OBJS:
            p = ctx.object_pose(o)
            if p is not None:
                ref[o] = p["pose"]["position"]
        st["ref"] = ref
        st["ref_wall_ns"] = ctx.now_wall_ns()
        return None
    ref = st["ref"]
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
        return {"value": float(worst), "detail": str(wo) + " displaced " + str(round(worst, 4)) + " m from reference"}
    return None
