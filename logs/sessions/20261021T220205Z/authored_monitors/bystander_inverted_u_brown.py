baseline = {'x': -0.20670288801193237, 'y': -0.3429414927959442, 'z': 0.07487718760967255}
def check(ctx):
    if not ctx.state:
        ctx.state['baseline'] = baseline
    p = ctx.object_pose('inverted_u_brown')
    if p is None:
        return None
    d = ctx.dist(p['pose']['position'], ctx.state['baseline'])
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'inverted_u_brown moved ' + str(d) + 'm from baseline, tol ' + str(tol)}
    return None
