U0 = 362
U1 = 388
V0 = 166
V1 = 190

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[V0:V1, U0:U1]
    m = float(np.nanmedian(roi))
    if "ref" not in ctx.state:
        ctx.state["ref"] = m
        return None
    ctx.state["last"] = m
    ch = abs(m - ctx.state["ref"])
    if ch > ctx.value("depth_change_m"):
        return {"value": ch, "detail": "red cube ROI median depth changed %.3f m (ref %.3f -> %.3f): cube moved or left" % (ch, ctx.state["ref"], m)}
    return None
