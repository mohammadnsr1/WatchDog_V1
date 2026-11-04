def check(ctx):
    for oid in ["base1", "base2"]:
        op = ctx.object_pose(oid)
        if op is None:
            continue
        p = op["pose"]["position"]
        if oid not in ctx.state:
            ctx.state[oid] = {"x": p["x"], "y": p["y"], "z": p["z"]}
            continue
        dd = ctx.dist(ctx.state[oid], p)
        if dd > ctx.value("trigger_m"):
            return {"value": dd, "detail": "%s moved %.4f m from run-start pose" % (oid, dd)}
    return None
