OBJ = "u_orange"

def check(ctx):
    held = ctx.held_object()
    ctx.state["held"] = held
    if held != OBJ:
        ctx.state["t0"] = None
        if "ref_d" in ctx.state:
            ctx.state.pop("ref_d")
        return None
    now = ctx.now_wall_ns()
    if ctx.state.get("t0") is None:
        ctx.state["t0"] = now
        return None
    if (now - ctx.state["t0"]) < ctx.value("settle_s") * 1e9:
        return None
    p = ctx.object_pose(OBJ)
    t = ctx.frame_pose("tcp")
    if p is None or t is None:
        return None
    d = ctx.dist(p["pose"]["position"], t["pose"]["position"])
    if "ref_d" not in ctx.state:
        ctx.state["ref_d"] = d
        return None
    dd = abs(d - ctx.state["ref_d"])
    if dd > ctx.value("slip_tol_m"):
        return {"value": dd, "detail": "u_orange-tcp distance changed while held"}
    return None
