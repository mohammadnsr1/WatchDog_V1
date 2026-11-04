def check(ctx):
    held = ctx.held_object()
    if held != "u_orange":
        return None
    t = ctx.transform("tcp", "u_orange")
    pos = t["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("slip_tol"):
        return {"value": d, "detail": "u_orange moved relative to gripper while held"}
    return None
