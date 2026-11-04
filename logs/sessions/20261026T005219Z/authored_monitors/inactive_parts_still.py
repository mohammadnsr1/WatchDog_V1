IDS = ["u_brown", "u_orange", "line_green", "inverted_u_yellow"]

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    settle_ns = 3000000000
    now = ctx.now_wall_ns()
    held = ctx.held_object()
    if "ref" not in ctx.state:
        ctx.state["ref"] = {}
        ctx.state["active"] = {}
        ctx.state["released_at"] = {}
    ref = ctx.state["ref"]
    active = ctx.state["active"]
    rel = ctx.state["released_at"]
    for o in IDS:
        p = ctx.object_pose(o)
        if p is None:
            continue
        pos = p["pose"]["position"]
        if o == held:
            active[o] = True
            continue
        if active.get(o):
            active[o] = False
            rel[o] = now
        if o in rel and now - rel[o] < settle_ns:
            ref[o] = pos
            continue
        if o not in ref:
            ref[o] = pos
            continue
        d = ctx.dist(pos, ref[o])
        if d > tol:
            return {"value": float(d), "detail": o + " (not held) displaced " + str(round(d, 4)) + " m from last rest pose"}
    return None
