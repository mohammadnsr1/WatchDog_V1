IDS = ["base1", "base2"]
def check(ctx):
    worst = 0.0
    who = ""
    for oid in IDS:
        p = ctx.object_pose(oid)
        if p is None:
            continue
        pos = p["pose"]["position"]
        if oid not in ctx.state:
            ctx.state[oid] = pos
            continue
        d = ctx.dist(pos, ctx.state[oid])
        if d > worst:
            worst = d
            who = oid
    if worst > ctx.value("trig_m"):
        return {"value": worst, "detail": who + " displaced %.4f m" % worst}
    return None
