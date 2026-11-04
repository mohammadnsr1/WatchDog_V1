baseline = {'x': -0.38477271795272827, 'y': -0.33969777822494507, 'z': 0.026932604610919952}
def check(ctx):
    if not ctx.state:
        ctx.state['baseline'] = baseline
    p = ctx.object_pose('u_green')
    if p is None:
        return None
    d = ctx.dist(p['pose']['position'], ctx.state['baseline'])
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'u_green moved ' + str(d) + 'm from baseline, tol ' + str(tol)}
    return None
