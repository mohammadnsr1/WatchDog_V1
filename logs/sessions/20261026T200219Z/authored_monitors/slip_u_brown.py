OBJ = "u_brown"

def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]

def rel(ctx):
    t = ctx.frame_pose("tcp")
    o = ctx.object_pose(OBJ)
    if t is None or o is None:
        return None
    if t["tf_stamp_ns"] != o["tf_stamp_ns"]:
        return None
    tp = t["pose"]["position"]
    op = o["pose"]["position"]
    q = t["pose"]["orientation"]
    v = [op["x"] - tp["x"], op["y"] - tp["y"], op["z"] - tp["z"]]
    u = [-q["x"], -q["y"], -q["z"]]
    w = q["w"]
    c = cross(u, v)
    tt = [2.0 * c[0], 2.0 * c[1], 2.0 * c[2]]
    c2 = cross(u, tt)
    return [v[i] + w * tt[i] + c2[i] for i in range(3)]

def check(ctx):
    tol = ctx.value("grasp_slip_tolerance_m")
    r = rel(ctx)
    if "ref" not in ctx.state:
        if r is not None:
            ctx.state["ref"] = r
            ctx.state["held_at_arm"] = ctx.held_object()
        return None
    if ctx.held_object() != OBJ:
        ctx.state["released"] = True
        return None
    if r is None:
        return None
    ref = ctx.state["ref"]
    d = math.sqrt(sum([(r[i] - ref[i]) ** 2 for i in range(3)]))
    if d > ctx.state.get("max_slip", 0.0):
        ctx.state["max_slip"] = d
    if d > tol:
        return {"value": d, "detail": OBJ + " moved " + str(round(d, 4)) + " m relative to the TCP while held"}
    return None
