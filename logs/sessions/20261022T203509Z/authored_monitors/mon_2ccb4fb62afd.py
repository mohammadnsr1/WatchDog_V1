TCP_FRAME = 'tcp'
OBJECT_ID = 'u_orange'

def check(ctx):
    held = ctx.held_object()
    if held != OBJECT_ID:
        ctx.state.pop('baseline', None)
        return None
    try:
        t = ctx.transform(TCP_FRAME, OBJECT_ID)
    except Exception:
        return None
    pos = t['translation']
    if 'baseline' not in ctx.state:
        ctx.state['baseline'] = pos
        return None
    base = ctx.state['baseline']
    d = ctx.dist(pos, base)
    tol = ctx.value('grasp_slip_tolerance_m')
    if d > tol:
        return {'value': d, 'detail': 'u_orange moved relative to the gripper while held (slip)'}
    return None
