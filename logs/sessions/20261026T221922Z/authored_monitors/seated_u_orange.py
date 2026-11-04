def check(ctx):
    op = ctx.object_pose("u_orange")
    if op is None:
        return None
    p = op["pose"]["position"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    dd = ctx.dist(ctx.state["ref"], p)
    if dd > ctx.value("tol"):
        return {"value": dd, "detail": "seated u_orange moved %.4f m from its seated pose" % dd}
    return None
