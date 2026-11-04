def check(ctx):
    img = ctx.rgb()
    if img is None:
        return None
    ctx.state["count"] = ctx.state.get("count", 0) + 1
    return None
