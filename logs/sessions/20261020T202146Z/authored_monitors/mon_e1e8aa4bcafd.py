OBJECT_ID = "u_orange"
def check(ctx):
    pose = ctx.object_pose(OBJECT_ID)
    if pose is None:
        return None
    pos = pose["pose"]["position"]
    if "baseline" not in ctx.state:
        ctx.state["baseline"] = pos
        return None
    b = ctx.state["baseline"]
    d = ((pos["x"]-b["x"])**2 + (pos["y"]-b["y"])**2 + (pos["z"]-b["z"])**2) ** 0.5
    if d > ctx.value("disturbance_tolerance_m"):
        return {"value": d, "detail": "u_orange (abandoned toppled) moved by " + str(d) + " m"}
    return None
