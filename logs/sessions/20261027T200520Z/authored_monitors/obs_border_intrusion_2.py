def strips(d):
    h = d.shape[0]
    w = d.shape[1]
    b = d[h - 30:h, :]
    r = d[int(h * 0.3):h - 30, w - 30:w]
    l = d[int(h * 0.7):h - 30, 0:30]
    return [b, r, l]

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    ss = strips(d)
    if "ref" not in ctx.state:
        ctx.state["ref"] = [np.array(s, copy=True) for s in ss]
        return None
    worst = 0.0
    for i in range(3):
        diff = ctx.state["ref"][i] - ss[i]
        closer = np.nan_to_num(diff, nan=0.0) > ctx.value("closer_m")
        frac = float(np.mean(closer))
        if frac > worst:
            worst = frac
    ctx.state["last_frac"] = worst
    if worst > ctx.value("frac_trigger"):
        return {"value": worst, "detail": "border-strip depth intrusion: %.3f of pixels nearer by > %.2f m" % (worst, ctx.value("closer_m"))}
    return None
