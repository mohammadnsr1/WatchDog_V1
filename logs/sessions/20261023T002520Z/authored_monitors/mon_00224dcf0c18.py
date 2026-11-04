def check(ctx):
    held = ctx.held_object()
    if held != "line_green":
        return None
    t = ctx.transform("tcp", "line_green")
    pos = t["translation"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    d = ctx.dist(ctx.state["baseline"], pos)
    tol = ctx.value("grasp_slip_tolerance_m")
    if d > tol:
        return {"value": d, "detail": "line_green slipped " + str(d) + "m in-hand, tol " + str(tol)}
    return None
