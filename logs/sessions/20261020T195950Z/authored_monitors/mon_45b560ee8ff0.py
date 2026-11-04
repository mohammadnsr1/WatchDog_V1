OBJ = "u_orange"
def check(ctx):
    if ctx.held_object() != OBJ:
        ctx.state.pop("ref", None)
        return None
    t = ctx.transform("tcp", OBJ)
    pos = t["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": OBJ + " moved " + str(d) + " m relative to the gripper while held"}
    return None
