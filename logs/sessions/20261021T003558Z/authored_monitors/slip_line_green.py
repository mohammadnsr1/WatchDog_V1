def check(ctx):
    held = ctx.held_object()
    if held != "line_green":
        ctx.state["ref"] = None
        return None
    rel = ctx.transform("tcp", "line_green")
    pos = rel["translation"]
    if ctx.state.get("ref") is None:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(ctx.state["ref"], pos)
    if d > ctx.value("slip_tol"):
        return {"value": d, "detail": "line_green shifted relative to gripper (slip) by " + str(d) + " m while held"}
    return None