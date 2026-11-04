def check(ctx):
    rgb = ctx.rgb()
    if rgb is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = rgb.copy()
        return None
    ref = ctx.state["ref"]
    diff = np.abs(rgb.astype(np.float32) - ref.astype(np.float32))
    changed = float(np.mean(diff))
    ctx.state["ref"] = rgb.copy()
    if changed > ctx.value("change_thresh"):
        return {"value": changed, "detail": "scene-wide rgb change above agent-chosen watch threshold"}
    return None
