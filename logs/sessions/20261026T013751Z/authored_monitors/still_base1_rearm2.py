def check(ctx):
    p = ctx.object_pose("base1")
    if p is None:
        return None
    pos = p["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "base1 displaced " + str(round(d, 4)) + " m from its post-shove rest pose"}
    return None
