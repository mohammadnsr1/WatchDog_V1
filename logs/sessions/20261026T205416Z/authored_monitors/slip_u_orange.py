OBJ = "u_orange"

def check(ctx):
    if ctx.state.get("done"):
        return None
    now = ctx.now_wall_ns()
    held = ctx.held_object()
    if held != OBJ:
        if ctx.state.get("held_since", 0) > 0:
            ctx.state["done"] = True
            ctx.state["released_ns"] = now
        return None
    if ctx.state.get("held_since", 0) == 0:
        ctx.state["held_since"] = now
        return None
    if now - ctx.state["held_since"] < ctx.value("settle_s") * 1e9:
        return None
    try:
        tr = ctx.transform("tcp", OBJ)["translation"]
    except ValueError:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = tr
        ctx.state["ref_ns"] = now
        return None
    d = ctx.dist(tr, ctx.state["ref"])
    if "maxd" not in ctx.state or d > ctx.state["maxd"]:
        ctx.state["maxd"] = d
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(d), "detail": "%s moved %.4f m relative to tcp while held (ref in tcp x=%.4f y=%.4f z=%.4f, now x=%.4f y=%.4f z=%.4f)" % (OBJ, d, ctx.state["ref"]["x"], ctx.state["ref"]["y"], ctx.state["ref"]["z"], tr["x"], tr["y"], tr["z"])}
    return None
