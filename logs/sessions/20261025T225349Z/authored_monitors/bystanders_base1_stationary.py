OBJS = ["u_green", "inverted_u_brown", "hex_blue", "hex_red", "base2", "base1"]

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
            return {"value": d, "detail": o + " moved " + str(round(d, 4)) + " m from its run-start pose"}
    return None
