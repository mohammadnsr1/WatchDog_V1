def check(ctx):
    held = ctx.held_object()
    if held != "inverted_u_yellow":
        return None
    t = ctx.transform("tcp", "inverted_u_yellow")
    pos = t["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("slip_tol"):
        return {"value": d, "detail": "inverted_u_yellow moved relative to gripper while held"}
    return None
