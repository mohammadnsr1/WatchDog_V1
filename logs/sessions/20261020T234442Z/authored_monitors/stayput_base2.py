def check(ctx):
    pose = ctx.object_pose("base2")
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(p, ref)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "base2 (other assembly's fixture) displaced from its resting pose"}
    return None
