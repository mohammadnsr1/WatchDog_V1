OBJ = "inverted_u_yellow"
def check(ctx):
    held = ctx.held_object()
    if held != OBJ:
        return None
    rel = ctx.transform("tcp", OBJ)
    t = rel["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": t["x"], "y": t["y"], "z": t["z"]}
        return None
    d = ctx.dist(t, ctx.state["ref"])
    if d > ctx.value("slip_m"):
        return {"value": d, "detail": "inverted_u_yellow shifted relative to the gripper while held"}
    return None