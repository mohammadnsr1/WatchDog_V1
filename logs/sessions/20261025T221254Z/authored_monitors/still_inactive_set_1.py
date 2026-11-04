OBJS = ["u_orange", "line_green", "inverted_u_yellow"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for o in OBJS:
            p = ctx.object_pose(o)
            if p is not None:
                ref[o] = p["pose"]["position"]
        ctx.state["ref"] = ref
        return None
    ref = ctx.state["ref"]
    for o in OBJS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        if o not in ref:
            ref[o] = p["pose"]["position"]
            continue
        d = ctx.dist(ref[o], p["pose"]["position"])
        if d > tol:
            return {"value": d, "detail": o + " displaced " + str(round(d, 4)) + " m from its rest pose"}
    return None
