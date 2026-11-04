def check(ctx):
    held = ctx.held_object()
    if held != "inverted_u_yellow":
        ctx.state["ref"] = None
        return None
    rel = ctx.transform("tcp", "inverted_u_yellow")
    pos = rel["translation"]
    if ctx.state.get("ref") is None:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(ctx.state["ref"], pos)
    if d > ctx.value("slip_tol"):
        return {"value": d, "detail": "inverted_u_yellow shifted relative to gripper (slip) by " + str(d) + " m while held"}
    return None