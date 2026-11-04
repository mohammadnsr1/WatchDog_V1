U0 = 165
U1 = 325
V0 = 255
V1 = 298

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[V0:V1, U0:U1]
    if "ref" not in ctx.state:
        ctx.state["ref"] = np.array(roi, copy=True)
        return None
    diff = np.nan_to_num(ctx.state["ref"] - roi, nan=0.0)
    frac = float(np.mean(diff > ctx.value("closer_m")))
    ctx.state["last_frac"] = frac
    if frac > ctx.value("frac_trigger"):
        return {"value": frac, "detail": "slab region: %.3f of pixels nearer by > %.2f m (object re-entering over inverted_u_yellow area)" % (frac, ctx.value("closer_m"))}
    return None
