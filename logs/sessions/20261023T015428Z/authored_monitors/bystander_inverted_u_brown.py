def check(ctx):
    if 'ref' not in ctx.state:
        p = ctx.object_pose(OBJ)
        ctx.state['ref'] = p['pose']['position'] if p else None
        return None
    ref = ctx.state['ref']
    if ref is None:
        return None
    p = ctx.object_pose(OBJ)
    if p is None:
        return None
    d = ctx.dist(ref, p['pose']['position'])
    if d > ctx.value('tol'):
        return {'value': d, 'detail': OBJ + ' moved ' + str(d) + ' m from its reference pose'}
    return None
OBJ = 'inverted_u_brown'
