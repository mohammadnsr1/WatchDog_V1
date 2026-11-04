def check(ctx):
    op = ctx.object_pose("hex_blue")
    if op is None:
        return None
    p = op["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    dd = ctx.dist(ctx.state["ref"], p)
    if dd > ctx.value("tol"):
        return {"value": dd, "detail": "hex_blue moved %.4f m from rest pose" % dd}
    return None
