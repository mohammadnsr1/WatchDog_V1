def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    roi = d[160:198, 355:395]
    if "ref" not in ctx.state:
        ctx.state["ref"] = roi.copy()
        ctx.state["ref_min_depth"] = float(np.nanmin(roi))
        ctx.state["ref_max_depth"] = float(np.nanmax(roi))
        return None
    ch = np.abs(roi - ctx.state["ref"]) > ctx.value("depth_change_m")
    n = int(np.count_nonzero(ch))
    if n > ctx.value("changed_px"):
        return {"value": float(n), "detail": "depth changed around the unlisted block at pixel (375,178): it moved, left, or something passed over it"}
    return None
