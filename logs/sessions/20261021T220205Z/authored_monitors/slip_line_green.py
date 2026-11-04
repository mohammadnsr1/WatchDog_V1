def check(ctx):
    held = ctx.held_object()
    if held != 'line_green':
        return None
    if not ctx.state:
        p = ctx.object_pose('line_green')
        tcp = ctx.frame_pose('tcp')
        if p is None or tcp is None:
            return None
        ctx.state['obj0'] = p['pose']['position']
        ctx.state['tcp0'] = tcp['pose']['position']
        return None
    p = ctx.object_pose('line_green')
    tcp = ctx.frame_pose('tcp')
    if p is None or tcp is None:
        return None
    rel_now = {'x': p['pose']['position']['x'] - tcp['pose']['position']['x'],
               'y': p['pose']['position']['y'] - tcp['pose']['position']['y'],
               'z': p['pose']['position']['z'] - tcp['pose']['position']['z']}
    rel0 = {'x': ctx.state['obj0']['x'] - ctx.state['tcp0']['x'],
            'y': ctx.state['obj0']['y'] - ctx.state['tcp0']['y'],
            'z': ctx.state['obj0']['z'] - ctx.state['tcp0']['z']}
    d = ctx.dist(rel_now, rel0)
    tol = ctx.value('tol')
    if d > tol:
        return {'value': d, 'detail': 'line_green shifted ' + str(d) + 'm relative to gripper, slip tol ' + str(tol)}
    return None
