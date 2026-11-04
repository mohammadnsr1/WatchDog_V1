def check(ctx):
    interval = ctx.value("interval_s")
    now = ctx.now_wall_ns()
    last = ctx.state.get("last", now)
    ctx.state["last"] = now
    if last == now:
        return None
    if now - last < interval * 1e9:
        return None
    return {"value": interval, "detail": "periodic scene check tick"}
