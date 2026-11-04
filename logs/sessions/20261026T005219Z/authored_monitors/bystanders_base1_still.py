IDS = ["u_green", "inverted_u_brown", "hex_blue", "hex_red", "base2", "base1"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for o in IDS:
            p = ctx.object_pose(o)
            if p is not None:
                ref[o] = p["pose"]["position"]
        ctx.state["ref"] = ref
        return None
    ref = ctx.state["ref"]
    for o in IDS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        if o not in ref:
            ref[o] = p["pose"]["position"]
            continue
        d = ctx.dist(p["pose"]["position"], ref[o])
        if d > tol:
            return {"value": float(d), "detail": o + " displaced " + str(round(d, 4)) + " m from rest pose"}
    return None
