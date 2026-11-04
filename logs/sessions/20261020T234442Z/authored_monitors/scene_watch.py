def check(ctx):
    rgb = ctx.rgb()
    if rgb is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = rgb.copy()
        return None
    ref = ctx.state["ref"]
    if ref.shape != rgb.shape:
        ctx.state["ref"] = rgb.copy()
        return None
    diff = np.abs(rgb.astype(np.float32) - ref.astype(np.float32))
    changed = float(np.mean(diff))
    if changed > ctx.value("change_level"):
        ctx.state["ref"] = rgb.copy()
        return {"value": changed, "detail": "workspace scene changed beyond expected arm/part motion baseline"}
    return None
