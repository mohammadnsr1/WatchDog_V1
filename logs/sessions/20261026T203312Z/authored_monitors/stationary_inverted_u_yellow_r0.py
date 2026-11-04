OBJ = "inverted_u_yellow"

def check(ctx):
    tol = ctx.value("disturbance_tolerance_m")
    st = ctx.state
    p = ctx.object_pose(OBJ)
    if p is None:
        return None
    if "ref" not in st:
        st["ref"] = p["pose"]["position"]
        st["ref_wall_ns"] = p["wall_ns"]
        return None
    d = ctx.dist(p["pose"]["position"], st["ref"])
    if d > tol:
        return {"value": float(d), "detail": OBJ + " displaced " + str(round(d, 4)) + " m from rest pose while inactive"}
    return None
