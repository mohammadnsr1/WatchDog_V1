def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[158:198, 352:398]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        return None
    diff = np.abs(roi - ctx.state["ref"])
    changed = float(np.nanmean(diff))
    if changed > ctx.value("change_m"):
        return {"value": changed, "detail": "depth changed in the ROI around the unlisted block at world ~(0.09,-0.38)"}
    return None
