def check(ctx):
    ids = ["u_green","inverted_u_brown","hex_blue","hex_red","base2"]
    tols = {"u_green": ctx.value("tol_u_green"), "inverted_u_brown": ctx.value("tol_inverted_u_brown"),
            "hex_blue": ctx.value("tol_hex_blue"), "hex_red": ctx.value("tol_hex_red"),
            "base2": ctx.value("tol_base2")}
    if "init" not in ctx.state:
        init = {}
        for oid in ids:
            p = ctx.object_pose(oid)
            if p is not None:
                init[oid] = p["pose"]["position"]
        ctx.state["init"] = init
        return None
    init = ctx.state["init"]
    for oid in ids:
        p = ctx.object_pose(oid)
        if p is None or oid not in init:
            continue
        d = ctx.dist(p["pose"]["position"], init[oid])
        if d > tols[oid]:
            return {"value": d, "detail": oid + " moved beyond tolerance"}
    return None
