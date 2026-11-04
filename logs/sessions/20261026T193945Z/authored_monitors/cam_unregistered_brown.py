def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[265:340, 550:615]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        ctx.state["ref_mean"] = float(np.nanmean(roi))
        return None
    diff = np.abs(roi - ctx.state["ref"])
    changed = float(np.nanmean(diff))
    if changed > ctx.value("change_m"):
        return {"value": changed, "detail": "depth changed in the ROI around the unregistered brown part at world ~(-0.50,-0.02)"}
    return None
