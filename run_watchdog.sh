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

# MODEL and EFFORT come from the environment, defaulting to the values in
# watchdog.settings.json ("model" / "effortLevel"), and are passed as explicit
# --model/--effort flags -- the highest-precedence form the installed claude
# CLI supports, so nothing in the user's global settings can silently override
# what this launch line says. The study varies ONLY the WatchDog's effort;
# override per run with e.g.:  EFFORT=high ./run_watchdog.sh s2 1
MODEL="${MODEL:-$(python3 -c "import json; print(json.load(open('watchdog.settings.json')).get('model','claude-sonnet-5'))")}"
EFFORT="${EFFORT:-$(python3 -c "import json; print(json.load(open('watchdog.settings.json')).get('effortLevel','low'))")}"

# Put the launch parameters in the run record. The manifest is written by the
# bringup before any session exists, so the launcher is the first place that
# knows which model and effort this role actually ran with.
session_dir=$(cat ~/.watchdog/current_session 2>/dev/null || true)
if [ -n "$session_dir" ] && [ -d "$session_dir" ]; then
  python3 - "$session_dir" "$scenario" "$assembly" "$MODEL" "$EFFORT" <<'PYEOF'
import datetime, json, subprocess, sys
session_dir, scenario, assembly, model, effort = sys.argv[1:6]
try:
    cli = subprocess.run(["claude", "--version"], capture_output=True,
                         text=True, timeout=15).stdout.strip()
except Exception as exc:
    cli = f"unavailable: {exc}"
record = {
    "role": "watchdog", "scenario": scenario, "assembly": assembly,
    "model": model, "effort": effort, "claude_cli_version": cli,
    "launched_wall_time_iso": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
path = f"{session_dir}/agent_config_watchdog.json"
with open(path, "w") as handle:
    json.dump(record, handle, indent=2)
print(f"wrote {path}", file=sys.stderr)
PYEOF
else
  echo "WARNING: ~/.watchdog/current_session does not name a directory;" \
       "agent_config_watchdog.json not written (bring the cell up first)" >&2
fi

echo "launching watchdog session: $config  (model=$MODEL effort=$EFFORT)" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --model "$MODEL" --effort "$EFFORT" \
  --settings watchdog.settings.json \
  'Call get_rendered_task(part="system_prompt") and then get_rendered_task(part="task"). Load system_prompt as your standing instructions and task as your opening task, then begin your role.'
