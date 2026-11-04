OBJ = "inverted_u_brown"
def check(ctx):
    pose = ctx.object_pose(OBJ)
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    d = ctx.dist(p, ctx.state["ref"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": "inverted_u_brown displaced from its rest pose"}
    return None
