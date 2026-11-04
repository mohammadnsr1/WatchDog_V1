OBJECT_ID = "u_brown"
def check(ctx):
    held = ctx.held_object()
    if held != OBJECT_ID:
        return None
    op = ctx.object_pose(OBJECT_ID)
    if op is None:
        return None
    try:
        tf = ctx.transform("tcp", OBJECT_ID)
    except Exception:
        return None
    t = tf["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = t
        return None
    ref = ctx.state["ref"]
    d = ((t["x"]-ref["x"])**2 + (t["y"]-ref["y"])**2 + (t["z"]-ref["z"])**2) ** 0.5
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": "u_brown moved relative to gripper (TCP) while held, indicating slip"}
    return None
