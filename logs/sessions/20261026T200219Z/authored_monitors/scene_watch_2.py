def roi(d):
    a = d[125:355, 380:635]
    b = d[330:358, 60:380]
    c = d[255:300, 225:395]
    return a, b, c

def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    a, b, c = roi(d)
    if "ra" not in ctx.state:
        ctx.state["ra"] = a.copy()
        ctx.state["rb"] = b.copy()
        ctx.state["rc"] = c.copy()
        return None
    t = ctx.value("depth_change_m")
    n = int(np.count_nonzero(np.abs(a - ctx.state["ra"]) > t))
    n = n + int(np.count_nonzero(np.abs(b - ctx.state["rb"]) > t))
    n = n + int(np.count_nonzero(np.abs(c - ctx.state["rc"]) > t))
    ctx.state["last_changed_px"] = n
    if n > ctx.value("changed_px"):
        return {"value": float(n), "detail": "depth changed in the base2/bystander region, the bottom table edge, or the flat-panel patch"}
    return None
