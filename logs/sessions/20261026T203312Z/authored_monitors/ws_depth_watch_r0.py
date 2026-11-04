def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[120:360, 330:640]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        ctx.state["ref_valid_px"] = int(np.sum(np.isfinite(roi)))
        ctx.state["ref_median_m"] = float(np.nanmedian(roi))
        return None
    diff = np.abs(roi - ctx.state["ref"])
    n = float(roi.size)
    changed = float(np.sum(diff > ctx.value("pixel_change_m"))) / n
    if changed > ctx.value("changed_fraction"):
        return {"value": changed, "detail": "depth changed over right-half bench ROI"}
    return None
