def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    rois = {"bottom": d[335:360, 30:610], "left": d[130:360, 0:28], "right": d[130:360, 612:640]}
    if "ref" not in ctx.state:
        ctx.state["ref"] = {k: v.copy() for k, v in rois.items()}
        return None
    worst = 0.0
    wname = ""
    for k, v in rois.items():
        diff = np.abs(v - ctx.state["ref"][k])
        frac = float(np.nanmean(diff > ctx.value("depth_change_m")))
        if frac > worst:
            worst = frac
            wname = k
    if worst > ctx.value("changed_fraction"):
        return {"value": worst, "detail": "depth changed at workspace edge strip " + wname}
    return None
