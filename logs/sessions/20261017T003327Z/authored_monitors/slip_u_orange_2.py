OBJ = "u_orange"
def check(ctx):
    if ctx.held_object() != OBJ:
        return None
    try:
        t = ctx.transform("tcp", OBJ)
    except Exception:
        return None
    p = t["translation"]
    if "ref" not in ctx.state:
        ctx.state["ref"] = {"x": p["x"], "y": p["y"], "z": p["z"]}
        return None
    d = ctx.dist(p, ctx.state["ref"])
    if d > ctx.value("tol"):
        return {"value": d, "detail": "u_orange moved relative to the gripper while held (reattempt)"}
    return None
