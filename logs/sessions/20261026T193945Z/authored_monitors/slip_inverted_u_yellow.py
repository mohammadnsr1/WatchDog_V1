PART = "inverted_u_yellow"

def rel(ctx):
    t = ctx.frame_pose("tcp")
    p = ctx.object_pose(PART)
    if t is None or p is None:
        return None
    return ctx.dist(t["pose"]["position"], p["pose"]["position"])

def check(ctx):
    if ctx.held_object() != PART:
        return None
    r = rel(ctx)
    if r is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = r
        return None
    drift = abs(r - ctx.state["ref"])
    if drift > ctx.value("grasp_slip_tolerance_m"):
        return {"value": drift, "detail": PART + " tcp distance changed by " + str(round(drift, 4)) + " m while held"}
    return None
