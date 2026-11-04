OBJS = ["base1", "base2"]

def check(ctx):
    if "ref" not in ctx.state:
        ref = {}
        for o in OBJS:
            s = ctx.object_pose(o)
            if s is None:
                return None
            p = s["pose"]["position"]
            ref[o] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        ctx.state["ref"] = ref
        return None
    worst = 0.0
    wo = ""
    for o in OBJS:
        s = ctx.object_pose(o)
        if s is None:
            continue
        d = ctx.dist(s["pose"]["position"], ctx.state["ref"][o])
        if d > worst:
            worst = d
            wo = o
    if worst > ctx.value("look_trigger_m"):
        return {"value": worst, "detail": wo + " moved %.4f m from run-start pose" % worst}
    return None
