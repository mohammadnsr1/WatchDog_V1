OBJ = "u_brown"

def check(ctx):
    p = ctx.object_pose(OBJ)
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        ctx.state["ref_wall_ns"] = p["wall_ns"]
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "%s displaced %.4f m from rest pose (ref x=%.4f y=%.4f z=%.4f, now x=%.4f y=%.4f z=%.4f)" % (OBJ, d, ctx.state["ref"]["x"], ctx.state["ref"]["y"], ctx.state["ref"]["z"], pos["x"], pos["y"], pos["z"])}
    return None
