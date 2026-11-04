baseline = {'x': 0.11455079913139343, 'y': 0.039382677525281906, 'z': 0.07487877458333969}
def check(ctx):
    if not ctx.state:
        ctx.state['baseline'] = baseline
    p = ctx.object_pose('hex_red')
    if p is None:
        return None
    d = ctx.dist(p['pose']['position'], ctx.state['baseline'])
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'hex_red moved ' + str(d) + 'm from baseline, tol ' + str(tol)}
    return None
