def check(ctx):
    if "n" not in ctx.state:
        ctx.state["n"] = 0
    ctx.state["n"] += 1
    return None
