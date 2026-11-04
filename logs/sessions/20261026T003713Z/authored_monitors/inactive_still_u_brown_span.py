OBJS = ["u_orange", "line_green", "inverted_u_yellow"]

def check(ctx):
    if "ref" not in ctx.state:
        ctx.state["ref"] = {}
    ref = ctx.state["ref"]
    for o in OBJS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        pos = p["pose"]["position"]
        if o not in ref:
            ref[o] = pos
            continue
        d = ctx.dist(pos, ref[o])
        if d > ctx.value("disturbance_m"):
            return {"value": d, "detail": o + " displaced " + str(round(d, 4)) + " m from rest"}
    return None
