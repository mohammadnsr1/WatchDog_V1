def check(ctx):
    out = None
    for oid in ["base1", "base2"]:
        p = ctx.object_pose(oid)
        if p is None:
            continue
        pos = p["pose"]["position"]
        if oid not in ctx.state:
            ctx.state[oid] = pos
            continue
        d = ctx.dist(pos, ctx.state[oid])
        if d > ctx.value("trigger_m") and out is None:
            out = {"value": d, "detail": oid + " moved %.4f m from run-start pose" % d}
    return out
