def check(ctx):
    oid = ctx.params["object_id_name"]["value"]
    pose = ctx.object_pose(oid)
    if pose is None:
        return None
    p = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(ref, p)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "fixture moved beyond tolerance; it must never be disturbed"}
    return None
