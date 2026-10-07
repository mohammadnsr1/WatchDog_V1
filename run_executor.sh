#!/usr/bin/env bash
# Launch the executor Claude Code session for a scored run.
#
# Usage: ./run_executor.sh [1|2]        S1-S3 run (the default path)
#        ./run_executor.sh s4 [1|2]     S4 run (executor self-monitoring)
#   assembly  1 = fmb_assembly, 2 = fmb_assembly_2
# Default: 1
#
# Model and effort are NOT arguments: they live in executor.settings.json as
# "model" and "effortLevel", edited there before a run (see RUN.md step 4).
# Both are echoed below so the launch line records what the run used.
#
# The executor's config is the same in S1-S3; scenario selection lives
# entirely on the watchdog side (see run_watchdog.sh and RUN.md). Launch the
# watchdog first and wait for it to arm before running this.
#
# S4 is the one exception: there is NO watchdog session at all. "s4" selects
# executor_s4.mcp.json / executor_2_s4.mcp.json, which point the SAME servers
# at the _s4 task file and executor_agent_s4.yaml and add the executor-side
# monitor host (executor_monitor_mcp_server). The cell bringup must be running
# guard_mode:=unguarded, and no run_watchdog.sh is launched.
set -euo pipefail
cd "$(dirname "$0")"

scenario=""
if [ "${1:-}" = "s4" ]; then
  scenario="s4"
  shift
fi
assembly="${1:-1}"

case "$assembly" in
  1) config="executor${scenario:+_s4}.mcp.json" ;;
  2) config="executor_2${scenario:+_s4}.mcp.json" ;;
  *) echo "assembly must be 1 or 2 (got: $assembly)" >&2; exit 1 ;;
esac

# MODEL and EFFORT come from the environment, defaulting to the values in
# executor.settings.json ("model" / "effortLevel"), passed as explicit
# --model/--effort flags (see run_watchdog.sh). STUDY DESIGN: the executor is
# fixed at one model and effort for every run of a batch -- only the
# WatchDog's effort varies. Do not override these per run unless you are
# starting a new batch, and then change the settings file so the default
# moves with it.
MODEL="${MODEL:-$(python3 -c "import json; print(json.load(open('executor.settings.json')).get('model','claude-sonnet-5'))")}"
EFFORT="${EFFORT:-$(python3 -c "import json; print(json.load(open('executor.settings.json')).get('effortLevel','low'))")}"

session_dir=$(cat ~/.watchdog/current_session 2>/dev/null || true)
if [ -n "$session_dir" ] && [ -d "$session_dir" ]; then
  python3 - "$session_dir" "$assembly" "$MODEL" "$EFFORT" "$scenario" <<'PYEOF'
import datetime, hashlib, json, os, subprocess, sys
session_dir, assembly, model, effort, scenario = sys.argv[1:6]
try:
    cli = subprocess.run(["claude", "--version"], capture_output=True,
                         text=True, timeout=15).stdout.strip()
except Exception as exc:
    cli = f"unavailable: {exc}"
# Class 4: EXECUTOR_REGISTRY_PATH hands this session's servers a corrupted
# registry (executor-role processes only; see registry/loader.py). Recorded
# here so the run record names it even if no server ever loaded it.
override = os.environ.get("EXECUTOR_REGISTRY_PATH", "").strip() or None
override_sha = None
if override:
    try:
        override_sha = hashlib.sha256(open(override, "rb").read()).hexdigest()
    except OSError as exc:
        override_sha = f"unreadable: {exc}"
record = {
    # scenario is null in S1-S3 (those live entirely on the watchdog side,
    # and the executor's surface is identical across them) and "s4" on an S4
    # run, where the scenario lives entirely on the executor side.
    "role": "executor", "scenario": scenario or None, "assembly": assembly,
    "model": model, "effort": effort, "claude_cli_version": cli,
    "executor_registry_override": override,
    "executor_registry_override_sha256": override_sha,
    "launched_wall_time_iso": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
path = f"{session_dir}/agent_config_executor.json"
with open(path, "w") as handle:
    json.dump(record, handle, indent=2)
print(f"wrote {path}", file=sys.stderr)
PYEOF
else
  echo "WARNING: ~/.watchdog/current_session does not name a directory;" \
       "agent_config_executor.json not written (bring the cell up first)" >&2
fi

if [ -n "${EXECUTOR_REGISTRY_PATH:-}" ]; then
  echo "CLASS 4: executor registry override = $EXECUTOR_REGISTRY_PATH" >&2
fi
echo "launching executor session: $config  (model=$MODEL effort=$EFFORT)" >&2
exec claude --mcp-config "$config" --strict-mcp-config \
  --model "$MODEL" --effort "$EFFORT" \
  --settings executor.settings.json \
  'Call get_rendered_task. Load system_prompt as your standing instructions and task as your opening task, then begin your role.'
