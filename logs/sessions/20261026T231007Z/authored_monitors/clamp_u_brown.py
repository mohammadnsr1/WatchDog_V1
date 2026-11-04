OBJ = "u_brown"
JOINT = "finger_joint"

def check(ctx):
    if ctx.held_object() != OBJ:
        return None
    j = ctx.joint(JOINT)
    if j is None or j["effort"] is None:
        return None
    e = abs(float(j["effort"]))
    if e > ctx.value("max_clamp_effort"):
        return {"value": e, "detail": "finger_joint |effort| " + str(round(e, 3)) + " while holding " + OBJ}
    return None
