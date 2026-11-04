OBJS = ["u_green", "inverted_u_brown", "hex_blue", "hex_red", "base2", "base1"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    if "ref" not in ctx.state:
        ctx.state["ref"] = {}
    ref = ctx.state["ref"]
    worst = 0.0
    worst_id = None
    for oid in OBJS:
        p = ctx.object_pose(oid)
        if p is None:
            continue
        pos = p["pose"]["position"]
        if oid not in ref:
            ref[oid] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
            continue
        d = ctx.dist(pos, ref[oid])
        if d > worst:
            worst = d
            worst_id = oid
    if worst_id is not None and worst > tol:
        return {"value": float(worst), "detail": worst_id + " displaced " + str(round(worst, 4)) + " m from its rest pose"}
    return None
