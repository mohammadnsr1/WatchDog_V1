OBJ = "line_green"

def check(ctx):
    s = ctx.object_pose(OBJ)
    if s is None:
        return None
    p = s["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        ctx.state["ref_wall_ns"] = s["wall_ns"]
        return None
    d = ctx.dist(p, ctx.state["ref"])
    if d > ctx.value("position_m"):
        return {"value": d, "detail": OBJ + " (seated) moved %.4f m from seated pose (%.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f)" % (d, ctx.state["ref"]["x"], ctx.state["ref"]["y"], ctx.state["ref"]["z"], p["x"], p["y"], p["z"])}
    return None
