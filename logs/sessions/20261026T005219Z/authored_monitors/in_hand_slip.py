def check(ctx):
    tol = ctx.value("grasp_slip_tolerance_m")
    held = ctx.held_object()
    st = ctx.state
    if held != st.get("held"):
        st["held"] = held
        st["off"] = None
        st["z0"] = None
    if held is None:
        return None
    p = ctx.object_pose(held)
    t = ctx.frame_pose("tcp")
    if p is None or t is None:
        return None
    pp = p["pose"]["position"]
    tp = t["pose"]["position"]
    if st["z0"] is None:
        st["z0"] = pp["z"]
        return None
    if st["off"] is None:
        if pp["z"] - st["z0"] > 0.01:
            st["off"] = {"x": pp["x"] - tp["x"], "y": pp["y"] - tp["y"], "z": pp["z"] - tp["z"]}
        return None
    cur = {"x": pp["x"] - tp["x"], "y": pp["y"] - tp["y"], "z": pp["z"] - tp["z"]}
    d = ctx.dist(cur, st["off"])
    if d > tol:
        return {"value": float(d), "detail": held + " moved relative to tcp by " + str(round(d, 4)) + " m while held"}
    return None
