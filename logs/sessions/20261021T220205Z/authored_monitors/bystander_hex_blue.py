baseline = {'x': 0.7770687341690063, 'y': -0.04575907438993454, 'z': 0.07487867772579193}
def check(ctx):
    if not ctx.state:
        ctx.state['baseline'] = baseline
    p = ctx.object_pose('hex_blue')
    if p is None:
        return None
    d = ctx.dist(p['pose']['position'], ctx.state['baseline'])
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'hex_blue moved ' + str(d) + 'm from baseline, tol ' + str(tol)}
    return None
