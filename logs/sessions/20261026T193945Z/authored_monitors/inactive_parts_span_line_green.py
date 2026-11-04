IDS = ["u_brown", "u_orange", "inverted_u_yellow"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ref = {}
        for oid in IDS:
            p = ctx.object_pose(oid)
            if p is not None:
                ref[oid] = p["pose"]["position"]
        ctx.state["ref"] = ref
        return None
    ref = ctx.state["ref"]
    for oid in IDS:
        p = ctx.object_pose(oid)
        if p is None or oid not in ref:
            continue
        d = ctx.dist(p["pose"]["position"], ref[oid])
        if d > tol:
            return {"value": d, "detail": oid + " displaced " + str(round(d, 4)) + " m from its rest pose"}
    return None
