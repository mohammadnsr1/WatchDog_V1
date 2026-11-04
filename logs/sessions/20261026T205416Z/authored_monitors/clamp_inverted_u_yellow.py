OBJ = "inverted_u_yellow"

def check(ctx):
    j = ctx.joint("finger_joint")
    if j is None:
        return None
    e = abs(j["effort"])
    if "peak" not in ctx.state or e > ctx.state["peak"]:
        ctx.state["peak"] = e
    if e > ctx.value("max_clamp_effort"):
        return {"value": float(e), "detail": "finger_joint |effort| %.3f exceeds %s max_clamp_effort %.1f" % (e, OBJ, ctx.value("max_clamp_effort"))}
    return None
