def check(ctx):
    oid = ctx.value("object_id_name")
    pose = ctx.object_pose(oid)
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    ref = ctx.state["ref"]
    d = ctx.dist(pos, ref)
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": float(d), "detail": "bystander object moved beyond tolerance"}
    return None
