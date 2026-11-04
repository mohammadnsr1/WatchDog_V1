def check(ctx):
    pose = ctx.object_pose("hex_red")
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    ref = ctx.state["ref"]
    d = ((p["x"]-ref["x"])**2 + (p["y"]-ref["y"])**2 + (p["z"]-ref["z"])**2) ** 0.5
    if d > ctx.value("tol"):
        return {"value": d, "detail": "hex_red moved " + str(d) + "m from its rest pose"}
    return None
