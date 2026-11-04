OBJ = "inverted_u_yellow"
def check(ctx):
    pose = ctx.object_pose(OBJ)
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    d = ctx.dist(p, ctx.state["ref"])
    if d > ctx.value("disturbance_m"):
        return {"value": d, "detail": "inverted_u_yellow moved while not yet active (its pick step has not begun)"}
    return None