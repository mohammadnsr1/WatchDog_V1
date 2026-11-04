baseline = {'x': 0.40278002619743347, 'y': -0.19856706261634827, 'z': 0.022378889843821526}
def check(ctx):
    if not ctx.state:
        ctx.state['baseline'] = baseline
    p = ctx.object_pose('base1')
    if p is None:
        return None
    d = ctx.dist(p['pose']['position'], ctx.state['baseline'])
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'base1 moved ' + str(d) + 'm from baseline, agent-chosen watch threshold ' + str(tol)}
    return None
