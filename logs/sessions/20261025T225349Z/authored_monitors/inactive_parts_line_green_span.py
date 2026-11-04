OBJS = ["u_brown", "u_orange", "inverted_u_yellow"]

def check(ctx):
    tol = ctx.value("tol_m")
    for o in OBJS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        pos = p["pose"]["position"]
        key = "ref_" + o
        if key not in ctx.state:
            ctx.state[key] = pos
            continue
        d = ctx.dist(pos, ctx.state[key])
        if d > tol:
            return {"value": d, "detail": o + " moved " + str(round(d, 4)) + " m while inactive"}
    return None
