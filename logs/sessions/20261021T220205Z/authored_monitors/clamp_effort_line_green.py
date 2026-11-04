def check(ctx):
    j = ctx.joint('finger_joint')
    if j is None or j.get('effort') is None:
        return None
    eff = abs(j['effort'])
    limit = ctx.value('limit')
    held = ctx.held_object()
    if held != 'line_green':
        return None
    if eff > limit:
        return {'value': eff, 'detail': 'clamp effort ' + str(eff) + ' exceeds max_clamp_effort ' + str(limit) + ' while holding line_green'}
    return None
