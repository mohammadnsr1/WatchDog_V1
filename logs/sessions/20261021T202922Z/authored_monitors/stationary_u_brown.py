def check(ctx):
    OBJ = "u_brown"
    pose = ctx.object_pose(OBJ)
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    ref = ctx.state["ref"]
    dx = p["x"] - ref["x"]
    dy = p["y"] - ref["y"]
    dz = p["z"] - ref["z"]
    d = (dx*dx + dy*dy + dz*dz) ** 0.5
    if d > ctx.value("tol"):
        return {"value": d, "detail": "u_brown displaced " + str(d) + " m after being seated"}
    return None
