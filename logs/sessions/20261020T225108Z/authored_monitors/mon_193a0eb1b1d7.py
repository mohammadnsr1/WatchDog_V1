def check(ctx):
    obj_ids = ["u_brown","u_orange","line_green","inverted_u_yellow","base1","base2","u_green","inverted_u_brown","hex_blue","hex_red"]
    tol = ctx.value("disturbance_tolerance_m")
    held = ctx.held_object()
    prev_held = ctx.state.get("prev_held")
    refs = ctx.state.get("refs")
    if refs is None:
        refs = {}
        for oid in obj_ids:
            p = ctx.object_pose(oid)
            if p is not None:
                refs[oid] = p["pose"]["position"]
        ctx.state["refs"] = refs
        ctx.state["prev_held"] = held
        return None
    if prev_held is not None and held != prev_held:
        p = ctx.object_pose(prev_held)
        if p is not None:
            refs[prev_held] = p["pose"]["position"]
    ctx.state["prev_held"] = held
    for oid in obj_ids:
        if oid == held:
            continue
        p = ctx.object_pose(oid)
        if p is None:
            continue
        ref = refs.get(oid)
        if ref is None:
            refs[oid] = p["pose"]["position"]
            continue
        d = ctx.dist(ref, p["pose"]["position"])
        if d > tol:
            return {"value": float(d), "detail": oid + " moved beyond disturbance tolerance while not held"}
    return None
