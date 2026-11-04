OID = "u_orange"

def check(ctx):
    if ctx.held_object() != OID:
        return None
    j = ctx.joint("finger_joint")
    if j is None or j["effort"] is None:
        return None
    e = abs(j["effort"])
    if e > ctx.state.get("max", 0.0):
        ctx.state["max"] = e
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint effort " + str(round(e, 2)) + " exceeds max_clamp_effort while holding " + OID}
    return None
