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
    ctx.state["ref"] = rgb.copy()
    if changed > ctx.value("scene_change_level"):
        return {"value": changed, "detail": "large scene-wide RGB change detected"}
    return None
