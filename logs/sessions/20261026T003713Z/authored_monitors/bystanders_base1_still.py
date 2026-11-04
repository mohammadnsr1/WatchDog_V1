OBJS = ["base1", "u_green", "inverted_u_brown", "hex_blue", "hex_red", "base2"]

def check(ctx):
    if "ref" not in ctx.state:
        ctx.state["ref"] = {}
    ref = ctx.state["ref"]
    worst = 0.0
    worst_id = ""
    for o in OBJS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        pos = p["pose"]["position"]
        if o not in ref:
            ref[o] = pos
            continue
        d = ctx.dist(pos, ref[o])
        if d > worst:
            worst = d
            worst_id = o
    if worst > ctx.value("disturbance_m"):
        return {"value": worst, "detail": worst_id + " displaced " + str(round(worst, 4)) + " m from its run-start pose"}
    return None
