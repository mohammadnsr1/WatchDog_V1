
MOVE_TOL = 0.01
NEAR_TCP = 0.12

def check(ctx):
    st = ctx.state
    refs = st.setdefault("refs", {})
    tcp = ctx.frame_pose("tcp")
    tcp_pos = tcp["pose"]["position"] if tcp is not None else None
    held = ctx.held_object()
    for oid in ctx.object_ids():
        if oid.endswith("_placement") or oid.endswith("_reorient") or oid.endswith("_hole"):
            continue
        p = ctx.object_pose(oid)
        if p is None:
            continue
        pos = p["pose"]["position"]
        robot_may_move_it = (held == oid) or (
            tcp_pos is not None
            and math.hypot(tcp_pos["x"] - pos["x"], tcp_pos["y"] - pos["y"]) < NEAR_TCP
        )
        if oid not in refs or robot_may_move_it:
            refs[oid] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
            continue
        delta = ctx.dist(refs[oid], pos)
        if delta > ctx.value("move_tol_m"):
            ref = refs[oid]
            refs[oid] = {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
            detail = ('{"kind":"object_moved","object_id":"' + oid + '"'
                      + ',"x":' + str(round(pos["x"], 3)) + ',"y":' + str(round(pos["y"], 3))
                      + ',"z":' + str(round(pos["z"], 3))
                      + ',"from_x":' + str(round(ref["x"], 3)) + ',"from_y":' + str(round(ref["y"], 3))
                      + ',"delta_m":' + str(round(delta, 4))
                      + ',"since_wall_ns":' + str(int(p.get("wall_ns") or ctx.now_wall_ns())) + '}')
            return {"value": delta, "detail": detail}
    return None
