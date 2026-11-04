def check(ctx):
    rgb = ctx.rgb()
    if rgb is None:
        return None
    frame = rgb.astype(np.float32)
    if "ref" not in ctx.state:
        ctx.state["ref"] = frame.copy()
        return None
    diff = np.abs(frame - ctx.state["ref"])
    changed = float(np.mean(diff))
    ctx.state["ref"] = frame.copy()
    if changed > ctx.value("scene_change_floor"):
        return {"value": changed, "detail": "workspace camera image changed beyond the background-noise floor; look to characterize"}
    return None
