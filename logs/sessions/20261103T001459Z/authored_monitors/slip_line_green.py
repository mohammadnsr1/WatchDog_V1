OID = "line_green"
def rel(ctx):
    t = ctx.frame_pose("tcp")
    o = ctx.object_pose(OID)
    if t is None or o is None:
        return None
    tp = t["pose"]["position"]
    q = t["pose"]["orientation"]
    op = o["pose"]["position"]
    vx = op["x"] - tp["x"]
    vy = op["y"] - tp["y"]
    vz = op["z"] - tp["z"]
    x = -q["x"]
    y = -q["y"]
    z = -q["z"]
    w = q["w"]
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return {"x": rx, "y": ry, "z": rz}
def check(ctx):
    if ctx.state.get("done"):
        return None
    if ctx.held_object() != OID:
        if "ref" in ctx.state:
            ctx.state["done"] = True
        return None
    r = rel(ctx)
    if r is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = r
        return None
    d = ctx.dist(r, ctx.state["ref"])
    if d > ctx.value("slip_tol_m"):
        return {"value": d, "detail": OID + " moved %.4f m relative to TCP while held" % d}
    return None
