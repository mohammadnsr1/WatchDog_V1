OID = "line_green"

def check(ctx):
    p = ctx.object_pose(OID)
    t = ctx.frame_pose("tcp")
    if p is None or t is None:
        return None
    pp = p["pose"]["position"]
    tp = t["pose"]["position"]
    if "z0" not in ctx.state:
        ctx.state["z0"] = pp["z"]
        return None
    if ctx.held_object() != OID:
        ctx.state.pop("ref", None)
        return None
    if "ref" not in ctx.state:
        if pp["z"] - ctx.state["z0"] < ctx.value("ref_capture_lift_m"):
            return None
        ctx.state["ref"] = ctx.dist(pp, tp)
        return None
    d = abs(ctx.dist(pp, tp) - ctx.state["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": d, "detail": "line_green-to-tcp distance changed while held"}
    return None
