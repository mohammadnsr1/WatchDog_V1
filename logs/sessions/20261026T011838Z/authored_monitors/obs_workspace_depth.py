JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "finger_joint"]

def robot_still(ctx):
    for j in JOINTS:
        w = ctx.joint_window(j, ctx.value("still_window_s"))
        if len(w) < 2:
            return False
        ps = np.array([s["position"] for s in w])
        if float(ps.max() - ps.min()) > ctx.value("still_rad"):
            return False
    return True

def check(ctx):
    if "armed" not in ctx.state:
        ctx.state["armed"] = True
        return None
    d = ctx.depth()
    if d is None:
        return None
    if not robot_still(ctx):
        if "ref" in ctx.state:
            ctx.state.pop("ref")
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = d.copy()
        return None
    diff = np.abs(d - ctx.state["ref"])
    n = int(np.sum(diff > ctx.value("change_m")))
    if n > ctx.value("min_pixels"):
        return {"value": float(n), "detail": "depth changed in " + str(n) + " pixels while the robot was stationary"}
    return None
