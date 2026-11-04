GRIPPER_JOINT = 'finger_joint'
OBJECT_ID = 'u_orange'

def check(ctx):
    held = ctx.held_object()
    if held != OBJECT_ID:
        return None
    j = ctx.joint(GRIPPER_JOINT)
    if j is None or j.get('effort') is None:
        return None
    eff = abs(j['effort'])
    limit = ctx.value('max_clamp_effort')
    if eff > limit:
        return {'value': eff, 'detail': 'clamp effort on u_orange exceeded registry max_clamp_effort'}
    return None
