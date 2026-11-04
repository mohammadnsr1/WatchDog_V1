def check(ctx):
    p = ctx.object_pose("u_brown")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "placed u_brown displaced " + str(round(d, 4)) + " m"}
    return None
