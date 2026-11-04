def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[164:192, 359:391]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        ctx.state["ref_min_m"] = float(np.nanmin(roi))
        ctx.state["ref_median_m"] = float(np.nanmedian(roi))
        return None
    diff = np.abs(roi - ctx.state["ref"])
    changed = float(np.sum(diff > ctx.value("pixel_change_m"))) / float(roi.size)
    if changed > ctx.value("changed_fraction"):
        return {"value": changed, "detail": "depth changed around unlisted block"}
    return None
