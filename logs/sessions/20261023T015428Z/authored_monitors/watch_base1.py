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
    if d > ctx.value('trigger_m'):
        return {'value': d, 'detail': OBJ + ' moved ' + str(d) + ' m from its reference pose (no registry tolerance; agent-chosen trigger)'}
    return None
OBJ = 'base1'
