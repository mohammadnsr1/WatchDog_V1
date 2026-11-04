def check(ctx):
    ids = ["u_green","base2","hex_blue","hex_red","inverted_u_brown","base1","u_brown","u_orange","inverted_u_yellow","line_green"]
    if "baseline" not in ctx.state:
        baseline = {}
        for oid in ids:
            p = ctx.object_pose(oid)
            if p is not None:
                baseline[oid] = p["pose"]["position"]
        ctx.state["baseline"] = baseline
        ctx.state["last_held"] = None
        return None
    held = ctx.held_object()
    last_held = ctx.state.get("last_held")
    if last_held is not None and held is None:
        p = ctx.object_pose(last_held)
        if p is not None:
            ctx.state["baseline"][last_held] = p["pose"]["position"]
    ctx.state["last_held"] = held
    thresh = ctx.value("disturbance_m")
    for oid in ids:
        if oid == held:
            continue
        p = ctx.object_pose(oid)
        if p is None:
            continue
        base = ctx.state["baseline"].get(oid)
        if base is None:
            continue
        d = ctx.dist(p["pose"]["position"], base)
        if d > thresh:
            return {"value": d, "detail": oid + " moved from last-known-rest baseline while not held (held=" + str(held) + ")"}
    return None
