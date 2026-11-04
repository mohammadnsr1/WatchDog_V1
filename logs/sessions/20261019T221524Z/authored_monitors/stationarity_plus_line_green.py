def check(ctx):
    refs = {
        "u_green": (-0.38477274775505066, -0.33969783782958984, 0.02693268470466137),
        "inverted_u_brown": (-0.20669963955879211, -0.34294071793556213, 0.07486759126186371),
        "hex_blue": (0.777072548866272, -0.04576914757490158, 0.07487958669662476),
        "hex_red": (0.11455002427101135, 0.03938165307044983, 0.07487878948450089),
        "base2": (-0.1825517863035202, -0.1053030788898468, 0.014129908755421638),
        "base1": (0.40278002619743347, -0.19856706261634827, 0.022378889843821526),
        "u_brown": (0.4039433002471924, -0.1486666202545166, 0.04337775707244873),
        "u_orange": (0.404045432806015, -0.24412383139133453, 0.04335569590330124),
        "line_green": (0.4049023985862732, -0.19735094904899597, 0.04583258926868439),
    }
    tol = ctx.value("disturbance_tolerance_m")
    worst = 0.0
    for obj, (rx, ry, rz) in refs.items():
        sample = ctx.object_pose(obj)
        if sample is None:
            continue
        p = sample["pose"]["position"]
        d = ((p["x"] - rx) ** 2 + (p["y"] - ry) ** 2 + (p["z"] - rz) ** 2) ** 0.5
        if d > worst:
            worst = d
        if d > tol:
            return {"value": d, "detail": obj + " moved " + str(d) + " m from resting pose"}
    ctx.state["worst"] = worst
    return None