#!/usr/bin/env bash
# Launch the executor Claude Code session for a scored run.
#
# Usage: ./run_executor.sh [1|2]
#   assembly  1 = fmb_assembly, 2 = fmb_assembly_2
# Default: 1
#
# The executor's config is the same in S1-S3; scenario selection lives
# entirely on the watchdog side (see run_watchdog.sh and RUN.md). Launch the
# watchdog first and wait for it to arm before running this.
set -euo pipefail
cd "$(dirname "$0")"

assembly="${1:-1}"

case "$assembly" in
  1) config="executor.mcp.json" ;;
  2) config="executor_2.mcp.json" ;;
  *) echo "assembly must be 1 or 2 (got: $assembly)" >&2; exit 1 ;;
esac

echo "launching executor session: $config" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --settings executor.settings.json \
  'Call get_rendered_task. Load system_prompt as your standing instructions and task as your opening task, then begin your role.'
