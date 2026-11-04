def check(ctx):
    pose = ctx.object_pose("hex_blue")
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = pos
        return None
    d = ctx.dist(pos, ctx.state["ref"])
    if d > ctx.value("disturbance_m"):
        return {"value": d, "detail": "hex_blue (bystander) moved " + str(d) + "m from rest"}
    return None
