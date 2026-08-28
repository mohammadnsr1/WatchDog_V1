# Scored-run procedure (Claude Code, two isolated sessions)

The ASU-provisioned Claude has no API key, so `run_agent.py` is not in the loop
for these runs. The model is driven by the Claude Code CLI; each role is one
terminal session with its own MCP config.

`run_agent.py` is unchanged and still works for the API-key path. This document
covers the Claude Code path only.

---

## The rule this whole setup exists to enforce

**The executor session loads only the command servers. The watchdog session
loads only the watchdog server.**

A watchdog that can command the arm is a second controller, not an independent
observer. The watchdog still sees everything the executor does, because
observation flows through the ROS event bus *into* the watchdog server
(`get_execution_state`, `get_events`, `get_evidence`, `wait_for_event`) — never
by holding the executor's tools.

Two things follow, and neither is optional:

- **`--strict-mcp-config` is mandatory.** Without it, `--mcp-config` *merges*
  with the project `.mcp.json` and the watchdog session silently regains motion,
  gripper and scene. Measured on this machine: 42 tools merged without the flag,
  1 with it. Nothing inside the session announces the merge — you only see it if
  you list tools.
- **Scored runs never use the VSCode extension window.** Extension sessions
  ignore CLI flags and always load the project `.mcp.json`, which contains all
  five servers. Use the extension for authoring only.

---

## Prerequisites

### 1. Standalone `claude` on PATH — not the extension binary

The extension ships a CLI at
`~/.vscode/extensions/anthropic.claude-code-<ver>-linux-x64/resources/native-binary/claude`.
Do **not** use it for scored runs: the path is pinned to one extension version
and moves out from under you on update (this machine currently has both 2.1.81
and 2.1.88 installed).

Install the standalone CLI and confirm:

```bash
command -v claude      # must resolve outside the .vscode extensions tree
claude --version       # e.g. 2.1.88 (Claude Code)
```

### 2. Simulation and MoveIt up, cell brought up

Isaac Sim playing, then the cell bringup:

```bash
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py
```

The bringup is the session owner. It resolves a fresh session directory,
creates it, and atomically writes `~/.watchdog/current_session` pointing at it.
Both Claude Code sessions then resolve that same directory by `rule='pointer'`.

**Do not export `WATCHDOG_SESSION_DIR` by hand.** A stale export in the
launching shell makes the *bringup* reuse an old session
(`_resolve_session_dir` honours it), which is the one way to get a correct-looking
launch pointing at last run's directory.

Verify the pointer names a directory that exists:

```bash
SD=$(cat ~/.watchdog/current_session); echo "$SD"; ls -d "$SD"
```

If it does not, fix that before launching either session. Neither config sets
`WATCHDOG_SESSION_DIR`; the two sessions agree *only* through this pointer. A
dangling pointer would otherwise send each session to its own fresh directory
and kill cross-agent correlation silently — so `get_rendered_task` hard-fails
when the resolved rule is `fresh` rather than letting the run proceed.

### 3. Record the CLI version into the run record

The manifest is written by the launch, before any session exists, so it cannot
know the CLI version. Record it alongside:

```bash
SD=$(cat ~/.watchdog/current_session)
claude --version > "$SD/claude_cli_version.txt"
```

---

## Launch — watchdog first, then executor

Order matters: the watchdog must be up and armed before the executor can depart
from anything worth observing.

### Terminal 1 — watchdog

```bash
cd ~/watchdog/WatchDog_V1
claude --mcp-config watchdog.mcp.json --strict-mcp-config
```

Then, as the session's first action:

1. Call `get_rendered_task`. It returns `system_prompt` and `task` as separate
   fields — load `system_prompt` as standing instructions and `task` as the
   opening task, before any other tool call.
2. Let it come up and **arm its monitors**. Wait for this before starting the
   executor.

### Terminal 2 — executor

```bash
cd ~/watchdog/WatchDog_V1
claude --mcp-config executor.mcp.json --strict-mcp-config
```

Then call `get_rendered_task` (hosted on the motion server for this role) and
load both returned fields the same way.

### Selecting the scenario

Both configs currently point at `fmb_assembly`. To run a different scenario,
edit the `EXECUTOR_TASK_PATH` / `WATCHDOG_TASK_PATH` values in the two config
files — they must name the *same scenario*, one `.executor.yaml` and one
`.watchdog.yaml`. Each server reads only its own role's variable, so a wrong
pairing gives one role the wrong task rather than leaking the other's file.

---

## Verify isolation before trusting the run

In each session, list the MCP tools:

- **Watchdog session** must contain **zero** command tools. If `move_tcp_to_pose`,
  `grasp`, `begin_step` or `lift` appear, `--strict-mcp-config` was missing or
  you are in an extension window. The run is not scoreable — relaunch.
- **Executor session** must contain **zero** watchdog tools (`arm_monitor`,
  `record_verdict`, `halt_execution`, `get_evidence`).
- **Watchdog still sees executor activity.** After the executor has made a call,
  `get_execution_state` on the watchdog side must show `last_executor_event`
  populated. This is the check that matters: it proves observation is arriving
  over the event bus rather than through tool access.
- **One session directory, not two.** Both roles' `get_rendered_task` returns
  `session_dir`; they must be identical, and:

```bash
SD=$(cat ~/.watchdog/current_session)
ls "$SD"    # events_*.jsonl for both roles + mcp_calls_executor.jsonl
            # + mcp_calls_watchdog.jsonl, all in ONE directory
```

---

## What the run produces

Launch-owned, unchanged by any of this:

- `events_motion.jsonl`, `events_gripper.jsonl`, `events_scene.jsonl`,
  `events_watchdog.jsonl` — per-process event logs
- the rosbag, `manifest.json`, `orchestrator/`
- the `/watchdog/events` topic as the unified cross-process timeline

Added for the Claude Code path:

- `mcp_calls_executor.jsonl` — every tool call made by the executor session,
  appended by all three of its server processes
- `mcp_calls_watchdog.jsonl` — the same for the watchdog session

Each line carries `wall_time_iso`, `server`, `tool`, `arguments`, `elapsed_sec`,
`ok`, `error`, `pid`. `wall_time_iso` is the cross-process correlation key, so
these align with `events_*.jsonl` by time. This is the action trace the harness
transcript used to hold. Chain-of-thought is deliberately not captured.

Verdict explanations now carry a summary of the decision basis — what was
observed, why it was or was not flagged, and for a fire what is believed to have
physically happened. That text flows into `events_watchdog.jsonl` through the
normal verdict path, which is how explanation quality stays scoreable without a
harness transcript.

---

## Optional: split permission sets

`executor.settings.json` and `watchdog.settings.json` hold per-role allowlists,
for reviewer-legible isolation:

```bash
claude --mcp-config watchdog.mcp.json --strict-mcp-config --settings watchdog.settings.json
```

This is belt-and-braces only. `--settings` is a separate flag from
`--mcp-config` and does not affect which servers load; an unloaded server's
tools cannot be called regardless of what any allowlist says. The shared
`.claude/settings.local.json` allowlist is harmless for the same reason.
