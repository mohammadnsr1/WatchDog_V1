#!/usr/bin/env bash
# Preflight for a scored run: prove the ROS graph has exactly one of everything.
#
# The failure this exists to catch is silent. Two Claude sessions (or a leftover
# one) each spawning robot_backend MCP servers put two nodes with the SAME name
# in the graph. ros2 warns once, then behaves unpredictably: duplicate
# /watchdog_node can both arm monitors and record verdicts, and both sets append
# to the same mcp_calls_*.jsonl, interleaving two sessions in one action trace.
#
# Usage:  scripts/preflight_check.sh          # report only
#         scripts/preflight_check.sh --kill   # also kill ORPHANED servers
#
# --kill only touches robot_backend MCP server processes whose parent is gone
# (PPID 1). It never kills a server owned by a live Claude session -- close that
# session instead, or the session loses its tools mid-run.
set -uo pipefail

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; OFF=$'\033[0m'
fail=0

echo "== 1. ROS node duplicates =="
if ! command -v ros2 >/dev/null 2>&1; then
  echo "  ${YEL}SKIP${OFF} ros2 not on PATH; source /opt/ros/jazzy/setup.bash and the workspace"
else
  dups=$(ros2 node list 2>/dev/null | sort | uniq -d | grep -Ev '^/(rviz2|transform_listener_impl)' || true)
  if [ -n "$dups" ]; then
    echo "  ${RED}FAIL${OFF} duplicate node names:"; echo "$dups" | sed 's/^/    /'
    fail=1
  else
    echo "  ${GRN}OK${OFF} no duplicate node names"
  fi
fi

echo "== 2. robot_backend MCP server processes =="
mapfile -t procs < <(pgrep -af "robot_backend\.mcp_servers|ros2 run robot_backend" 2>/dev/null || true)
if [ "${#procs[@]}" -eq 0 ]; then
  echo "  ${GRN}OK${OFF} none running"
else
  # Walk up past the `ros2 run` wrapper (and its venv child) to the process that
  # actually OWNS the server -- a Claude session, a terminal, or init if orphaned.
  # Without this every wrapper counts as its own owner and the count is meaningless.
  root_owner() {
    local pid=$1 ppid cmd
    for _ in 1 2 3 4 5; do
      ppid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
      [ -z "$ppid" ] || [ "$ppid" = "0" ] && { echo 1; return; }
      cmd=$(ps -o cmd= -p "$ppid" 2>/dev/null)
      case "$cmd" in
        *robot_backend*) pid=$ppid ;;
        *)               echo "$ppid"; return ;;
      esac
    done
    echo "$pid"
  }
  declare -A owners=()
  for line in "${procs[@]}"; do
    pid=${line%% *}
    own=$(root_owner "$pid")
    [ -z "$own" ] && continue
    owners["$own"]=$(( ${owners["$own"]:-0} + 1 ))
  done
  for ppid in "${!owners[@]}"; do
    cmd=$(ps -o cmd= -p "$ppid" 2>/dev/null | cut -c1-90)
    if [ "$ppid" = "1" ]; then
      echo "  ${RED}ORPHANED${OFF} ${owners[$ppid]} process(es) with no parent"
      fail=1
    else
      echo "  owner pid $ppid (${owners[$ppid]} procs): ${cmd:-<gone>}"
    fi
  done
  live=$(( ${#owners[@]} ))
  [ "$live" -gt 1 ] && { echo "  ${RED}FAIL${OFF} $live separate owners -- expect 1 per role, never 2 of the same"; fail=1; }
fi

echo "== 3. session pointer =="
ptr="$HOME/.watchdog/current_session"
if [ ! -f "$ptr" ]; then
  echo "  ${RED}FAIL${OFF} $ptr missing -- bringup has not run"; fail=1
else
  sd=$(cat "$ptr")
  if [ -d "$sd" ]; then
    echo "  ${GRN}OK${OFF} $sd"
  else
    echo "  ${RED}FAIL${OFF} pointer names a missing directory: $sd"; fail=1
  fi
fi

if [ "${1:-}" = "--kill" ]; then
  echo "== killing ORPHANED servers only =="
  n=0
  while read -r pid; do
    [ -z "$pid" ] && continue
    if [ "$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')" = "1" ]; then
      kill "$pid" 2>/dev/null && { echo "  killed $pid"; n=$((n+1)); }
    fi
  done < <(pgrep -f "robot_backend\.mcp_servers|ros2 run robot_backend" 2>/dev/null || true)
  echo "  $n orphan(s) killed"
fi

echo
[ "$fail" -eq 0 ] && echo "${GRN}PREFLIGHT PASS${OFF}" || echo "${RED}PREFLIGHT FAIL${OFF} -- do not score this run"
exit "$fail"
