OBJ = "u_orange"

def check(ctx):
    p = ctx.object_pose(OBJ)
    t = ctx.frame_pose("tcp")
    if p is None or t is None:
        return None
    op = p["pose"]["position"]
    tp = t["pose"]["position"]
    if "rest_z" not in ctx.state:
        ctx.state["rest_z"] = op["z"]
        return None
    if ctx.held_object() != OBJ:
        if "base" in ctx.state:
            ctx.state["released"] = True
        return None
    if ctx.state.get("released"):
        return None
    off = {"x": op["x"] - tp["x"], "y": op["y"] - tp["y"], "z": op["z"] - tp["z"]}
    if "base" not in ctx.state:
        if op["z"] - ctx.state["rest_z"] > ctx.value("lift_gate_m"):
            ctx.state["base"] = off
        return None
    d = ctx.dist(off, ctx.state["base"])
    if d > ctx.value("slip_m"):
        return {"value": d, "detail": "u_orange moved " + str(round(d, 4)) + " m relative to tcp while held"}
    return None
