def check(ctx):
    interval = ctx.value("interval_s")
    now = ctx.now_wall_ns()
    last = ctx.state.get("last", 0)
    if now - last < interval * 1e9:
        return None
    ctx.state["last"] = now
    return {"value": interval, "detail": "periodic scene check tick"}
