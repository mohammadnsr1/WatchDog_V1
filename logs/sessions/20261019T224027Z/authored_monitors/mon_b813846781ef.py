def check(ctx):
    held = ctx.held_object()
    if held != "line_green":
        return None
    t = ctx.transform("tcp", "line_green")
    pos = t["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(d), "detail": "line_green moved relative to gripper by " + str(d) + " m while held"}
    return None
