baseline = {'x': -0.1825517863035202, 'y': -0.1053030788898468, 'z': 0.014129908755421638}
def check(ctx):
    if not ctx.state:
        ctx.state['baseline'] = baseline
    p = ctx.object_pose('base2')
    if p is None:
        return None
    d = ctx.dist(p['pose']['position'], ctx.state['baseline'])
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'base2 moved ' + str(d) + 'm from baseline, agent-chosen watch threshold ' + str(tol)}
    return None
