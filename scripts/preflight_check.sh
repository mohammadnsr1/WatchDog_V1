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
  # Restart the daemon FIRST. `ros2 node list` reads the daemon's cached
  # discovery, and the daemon does not reliably prune a node that died while
  # other nodes stayed up -- so a node killed and relaunched (an rviz2 from an
  # earlier bringup, say) can linger and show as a duplicate that no longer
  # exists. Without this the check reports phantom failures, and worse, it can
  # mask a real one behind a name you have learned to ignore.
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  nodes=$(ros2 node list 2>/dev/null)
  dups=$(echo "$nodes" | sort | uniq -d | grep -Ev '^/transform_listener_impl' || true)
  if [ -n "$dups" ]; then
    echo "  duplicates after a daemon refresh (these are real, not cache):"
    while read -r n; do
      [ -z "$n" ] && continue
      # Count live processes plausibly backing this node, to separate "two
      # processes" from "one process registering the name twice".
      short=${n#/}
      nproc=$(ps -eo comm= 2>/dev/null | grep -cx "$short" || true)
      case "$n" in
        /rviz2)
          echo "    ${YEL}WARN${OFF} $n (rviz2 processes: $nproc) -- visualization only:"
          echo "           no monitors, no commands, no session logging. Not scoring-relevant."
          ;;
        *)
          echo "    ${RED}FAIL${OFF} $n (matching processes: $nproc)"; fail=1 ;;
      esac
    done <<< "$dups"
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
  live=0
  for ppid in "${!owners[@]}"; do
    cmd=$(ps -o cmd= -p "$ppid" 2>/dev/null | cut -c1-90)
    case "$cmd" in
      *claude*|*"ros2 run"*)
        echo "  owner pid $ppid (${owners[$ppid]} procs): ${cmd:-<gone>}"
        live=$((live+1)) ;;
      *)
        # Reparented to a subreaper (systemd --user) or init: the session that
        # started it is gone but the process survived, still holding its ROS
        # node name. A later session's server then collides and its tools never
        # register. Run with --kill.
        echo "  ${RED}ORPHANED${OFF} ${owners[$ppid]} process(es), reparented to: ${cmd:-<gone>}"
        echo "           -> rerun with --kill"
        fail=1 ;;
    esac
  done
  [ "$live" -gt 1 ] && { echo "  ${RED}FAIL${OFF} $live live owners -- expect 1 per role, never 2 of the same"; fail=1; }
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
  # An orphan is NOT identified by PPID 1. When a Claude session's MCP server
  # dies, the surviving python grandchild reparents to the nearest SUBREAPER,
  # which on a systemd user session is `systemd --user` (observed: PPID 2777),
  # never init. The earlier PPID==1 test therefore matched nothing and this
  # flag silently killed zero orphans while three hung servers squatted the
  # ROS node names -- which is what made an executor session come up with
  # gripper tools but no motion tools, so get_rendered_task was missing.
  #
  # Correct test: the owner is not a live claude session and not a `ros2 run`
  # wrapper. Also SIGKILL after SIGTERM -- these hang and ignore SIGTERM
  # (they stop answering discovery while still holding their names).
  n=0
  while read -r pid; do
    [ -z "$pid" ] && continue
    ppid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$ppid" ] && continue
    owner=$(ps -o cmd= -p "$ppid" 2>/dev/null)
    case "$owner" in
      *claude*|*"ros2 run"*) continue ;;   # live, leave alone
    esac
    kill "$pid" 2>/dev/null
    sleep 0.5
    if kill -0 "$pid" 2>/dev/null; then kill -9 "$pid" 2>/dev/null; fi
    kill -0 "$pid" 2>/dev/null || { echo "  killed $pid (was owned by: ${owner:-<gone>})"; n=$((n+1)); }
  done < <(pgrep -f "robot_backend\.mcp_servers" 2>/dev/null || true)
  echo "  $n orphan(s) killed"
fi

echo
[ "$fail" -eq 0 ] && echo "${GRN}PREFLIGHT PASS${OFF}" || echo "${RED}PREFLIGHT FAIL${OFF} -- do not score this run"
exit "$fail"
