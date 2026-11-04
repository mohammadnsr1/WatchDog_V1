def check(ctx):
    d = ctx.depth()
    if d is None:
        return None
    if "ref" not in ctx.state:
        ctx.state["ref"] = [row for row in d]
        return None
    ref = ctx.state["ref"]
    total = 0.0
    count = 0
    h = len(d)
    for i in range(0, h, 8):
        row = d[i]
        rrow = ref[i]
        w = len(row)
        for j in range(0, w, 8):
            a = row[j]
            b = rrow[j]
            if a == a and b == b:
                diff = a - b
                if diff < 0:
                    diff = -diff
                total = total + diff
                count = count + 1
    if count == 0:
        return None
    mean_diff = total / count
    if mean_diff > ctx.value("change_m"):
        return {"value": mean_diff, "detail": "workspace depth image changed beyond baseline, possible unexpected object/event"}
    return None