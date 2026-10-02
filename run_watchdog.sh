#!/usr/bin/env bash
# Launch the WatchDog Claude Code session for a scored run.
#
# Usage: ./run_watchdog.sh [s1|s2|s3] [1|2]
#   scenario  s1 (library), s2 (library + authored), s3 (authored only)
#   assembly  1 = fmb_assembly, 2 = fmb_assembly_2
# Defaults: s1 1
#
# Model and effort are NOT arguments: they live in watchdog.settings.json as
# "model" and "effortLevel", edited there before a run (see RUN.md step 4).
# Both are echoed below so the launch line records what the run used.
#
# Picks the matching watchdog*.mcp.json, always passes --strict-mcp-config and
# --settings watchdog.settings.json, and submits the bootstrap instruction as
# the session's first message so the operator does not type it by hand. The
# task is fetched in two parts because the combined get_rendered_task result
# (~50 KB) exceeds the CLI's MCP result-size cap and would arrive as a preview
# this Read-denied session cannot open -- see RUN.md.
set -euo pipefail
cd "$(dirname "$0")"

scenario="${1:-s1}"
assembly="${2:-1}"

case "$scenario" in
  s1) suffix="" ;;
  s2) suffix="_s2" ;;
  s3) suffix="_s3" ;;
  *) echo "scenario must be s1, s2 or s3 (got: $scenario)" >&2; exit 1 ;;
esac
case "$assembly" in
  1) config="watchdog${suffix}.mcp.json" ;;
  2) config="watchdog_2${suffix}.mcp.json" ;;
  *) echo "assembly must be 1 or 2 (got: $assembly)" >&2; exit 1 ;;
esac

# model and effortLevel live in watchdog.settings.json (edit them there before
# a run). Echoed here rather than passed as flags: the settings file is the
# single source of truth, and a --model/--effort flag would silently outrank
# it. Reading them back is what makes a typo visible -- an unrecognised key or
# value is otherwise ignored with only a warning, and the run would proceed on
# the account default with nothing in the terminal saying so.
params=$(python3 -c "
import json
d = json.load(open('watchdog.settings.json'))
print('model=%s effort=%s' % (d.get('model', '<unset: account default>'),
                              d.get('effortLevel', '<unset: model default>')))
")

echo "launching watchdog session: $config  ($params)" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --settings watchdog.settings.json \
  'Call get_rendered_task(part="system_prompt") and then get_rendered_task(part="task"). Load system_prompt as your standing instructions and task as your opening task, then begin your role.'
