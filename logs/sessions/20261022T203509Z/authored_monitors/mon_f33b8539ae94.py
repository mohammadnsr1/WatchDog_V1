OBJECT_ID = 'base2'

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
    tol = ctx.value('watch_threshold_m')
    if d > tol:
        return {'value': d, 'detail': 'base2 (other fixture) moved from its baseline pose'}
    return None
