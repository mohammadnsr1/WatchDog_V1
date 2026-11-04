OBJ = "u_orange"

def check(ctx):
    st = ctx.state
    if ctx.held_object() != OBJ:
        st["last_not_held_ns"] = ctx.now_wall_ns()
        return None
    t = ctx.transform("tcp", OBJ)["translation"]
    if "ref" not in st:
        st["ref"] = t
        st["ref_norm_m"] = math.sqrt(t["x"] ** 2 + t["y"] ** 2 + t["z"] ** 2)
        st["held"] = ctx.held_object()
        return None
    d = ctx.dist(t, st["ref"])
    if d > ctx.value("grasp_slip_tolerance_m"):
        return {"value": float(d), "detail": OBJ + " moved " + str(round(d, 4)) + " m relative to tcp while held"}
    return None
