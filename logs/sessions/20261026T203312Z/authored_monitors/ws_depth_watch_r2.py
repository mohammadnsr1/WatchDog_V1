def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[150:360, 400:640]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        ctx.state["ref_median_m"] = float(np.nanmedian(roi))
        return None
    diff = np.abs(roi - ctx.state["ref"])
    changed = float(np.sum(diff > ctx.value("pixel_change_m"))) / float(roi.size)
    if changed > ctx.value("changed_fraction"):
        return {"value": changed, "detail": "depth changed over bystander-area ROI"}
    return None
