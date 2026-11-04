def rel(ctx, oid):
    op = ctx.object_pose(oid)
    tp = ctx.frame_pose("tcp")
    if op is None or tp is None:
        return None
    p = op["pose"]["position"]
    t = tp["pose"]["position"]
    q = tp["pose"]["orientation"]
    vx = p["x"] - t["x"]
    vy = p["y"] - t["y"]
    vz = p["z"] - t["z"]
    qx = -q["x"]
    qy = -q["y"]
    qz = -q["z"]
    qw = q["w"]
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    cx2 = qy * cz - qz * cy
    cy2 = qz * cx - qx * cz
    cz2 = qx * cy - qy * cx
    return {"x": vx + 2.0 * (qw * cx + cx2), "y": vy + 2.0 * (qw * cy + cy2), "z": vz + 2.0 * (qw * cz + cz2)}

def check(ctx):
    if ctx.held_object() != "u_brown":
        return None
    r = rel(ctx, "u_brown")
    if r is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = r
        return None
    dd = ctx.dist(ctx.state["ref"], r)
    if dd > ctx.value("slip_tol"):
        return {"value": dd, "detail": "u_brown moved %.4f m relative to TCP while held" % dd}
    return None
