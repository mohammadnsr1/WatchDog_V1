OBJ = "line_green"
TCP = "tcp"
def check(ctx):
    held = ctx.held_object()
    if held != OBJ:
        return None
    op = ctx.object_pose(OBJ)
    tp = ctx.frame_pose(TCP)
    if op is None or tp is None:
        return None
    rel = ctx.dist(op["pose"]["position"], tp["pose"]["position"])
    if "ref" not in ctx.state:
        ctx.state["ref"] = rel
        return None
    drift = abs(rel - ctx.state["ref"])
    if drift > ctx.value("slip_m"):
        return {"value": drift, "detail": OBJ + " drifted relative to the gripper TCP while held"}
    return None
