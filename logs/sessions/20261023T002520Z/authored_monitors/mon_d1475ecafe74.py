def check(ctx):
    rgb = ctx.rgb()
    if rgb is None:
        return None
    if "armed" not in ctx.state:
        ctx.state["armed"] = True
        return None
    return {"value": 1.0, "detail": "periodic scene snapshot tick"}
