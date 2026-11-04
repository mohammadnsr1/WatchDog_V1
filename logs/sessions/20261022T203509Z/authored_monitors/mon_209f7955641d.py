OBJECT_ID = 'hex_red'

def check(ctx):
    pose = ctx.object_pose(OBJECT_ID)
    if pose is None:
        return None
    pos = pose['pose']['position']
    if 'baseline' not in ctx.state:
        ctx.state['baseline'] = pos
        return None
    base = ctx.state['baseline']
    d = ctx.dist(pos, base)
    tol = ctx.value('disturbance_tolerance_m')
    if d > tol:
        return {'value': d, 'detail': 'hex_red moved from its baseline resting pose'}
    return None
