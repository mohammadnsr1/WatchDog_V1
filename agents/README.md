# WatchDog / Executor Agent Harness

Two independent agents study the same assembly task from opposite sides:

- **Executor** — commands the robot through the `gripper`, `motion`, and
  `scene` MCP servers. Plans, calls skills, reports outcomes.
- **WatchDog** — an independent supervisory conformance monitor, reachable
  only through the `watchdog` MCP server. Observes the cell, arms
  deterministic monitors, halts on bound violations, records verdicts. It
  never commands the robot.

The runner (`agent_harness/runner.py`) is identical for both agents and
contains no agent-specific logic. An "agent" here is exactly three things: an
MCP server list, a system prompt, and a task rendering — see `config/` and
`prompts/`.

## Why they are separated

- **WatchDog must not import `robot_backend` or anything from `ros2_ws`.**
  Its only channel to the cell is its stdio MCP server. This harness lives at
  `WatchDog_V1/agents/`, outside `ros2_ws/`, and is not a ROS package.
- **The executor must never see the WatchDog server**, and must never learn a
  monitor exists or that it can be halted. Its config (`executor_agent.yaml`)
  lists only `gripper`/`motion`/`scene`. Its system prompt never mentions
  WatchDog, monitoring, or supervision. If the executor knew it was watched,
  the central result of the study would be invalid.
- **The runner never repairs, retries, reorders, or summarizes an agent's
  decisions.** For WatchDog, the tool sequence and verdicts are the
  experimental record. For the executor, a runner-side retry would make the
  success claim the runner's rather than the agent's. An agent may still
  retry its *own* actions through its own tool calls — that's the agent
  reasoning, not the runner intervening.

`agent_harness/config.py` warns loudly (and refuses to load) if a single
config lists the `watchdog` server alongside any executor server — that
combination would leak monitoring tools to the executor and is treated as a
configuration bug, not a valid setup.

## Setup

1. **Source the ROS overlay** before launching either agent — the MCP
   servers (`gripper_mcp_server`, `motion_mcp_server`, `scene_mcp_server`,
   `watchdog_mcp_server`) are ROS2 nodes and need the workspace on the
   Python path and `AMENT_PREFIX_PATH` set up:

   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/watchdog/WatchDog_V1/ros2_ws/install/setup.bash
   ```

2. **Install harness dependencies** (separate from the ROS Python
   environment if you use one):

   ```bash
   cd ~/watchdog/WatchDog_V1/agents
   pip install -r requirements.txt
   ```

3. **Copy `.env.example` to `.env` and fill in your keys:**

   ```bash
   cp .env.example .env
   # edit .env: ANTHROPIC_API_KEY=..., OPENAI_API_KEY=...
   ```

   `.env` is gitignored and must never be committed. Each agent config names
   the environment variable holding its own key via `api_key_env` — the two
   agents may use different providers and different keys. Never pass a key as
   a CLI argument; that puts it in shell history.

## Dry run first, always

Before running either agent for real, run `--dry-run` for **both** configs.
This opens the configured MCP servers, lists the tools actually discovered,
and prints the fully rendered system prompt and task — without calling the
provider at all.

```bash
python3 run_agent.py --config config/watchdog_agent.yaml  --task tasks/two_cube_stack.yaml --dry-run
python3 run_agent.py --config config/executor_agent.yaml  --task tasks/two_cube_stack.yaml --dry-run
```

Use this to confirm, every time you change a config or prompt:

- The **executor's** dry-run tool list contains only `gripper`/`motion`/
  `scene` tools — no `watchdog` server, no `wait_for_event`, no monitor
  tools of any kind.
- The **WatchDog** dry-run tool list includes `available_monitor_definitions`
  and a `wait_for_event` that advertises `event_types`, alongside
  `arm_monitor`, `record_verdict`, etc.
- No `{{PLACEHOLDER}}` token survives into the rendered task text for either
  agent. `run_agent.py` raises instead of silently passing a literal
  placeholder to the model.

## Launch protocol for a paired run

The two agents are launched as **two separate processes** against the same
task file:

```bash
python3 run_agent.py --config config/watchdog_agent.yaml --task tasks/two_cube_stack.yaml
python3 run_agent.py --config config/executor_agent.yaml --task tasks/two_cube_stack.yaml
```

**Fixed protocol for this study: start WatchDog first, then the executor.**
WatchDog must be armed and watching (or at minimum, its process live and
listening on its event bus) before the executor's first `begin_step` event is
published, or WatchDog can miss the opening of step 1 entirely. There is no
supervisor process that launches both together — coupling their lifecycles in
code would blur the independence the study rests on. Launching them is a
manual, two-terminal (or two-job) operation every time, by design.

`--provider` and `--model` CLI flags override the config file if you need to
swap providers for a single run without editing YAML.

## Transcripts

Every run writes a JSONL transcript to `transcripts/` (gitignored, created at
runtime), one record per line, flushed on write so a crashed or halted run
stays readable. Filename pattern:

```
<agent>_<task_name>_<utc_iso>.jsonl
```

e.g. `watchdog_two_cube_stack_20260720T140512Z.jsonl` and
`executor_two_cube_stack_20260720T140508Z.jsonl` — a paired executor/WatchDog
run over the same task sorts together (same `<task_name>`) and is told apart
by the `<agent>_` prefix. Every record carries `wall_ns` on the same axis as
the cell's own event and monitor-fire timestamps, which is what lets you
correlate an agent's turn timing against physical events after the fact —
and, for WatchDog, is the evidence that a halt never waited on the agent.

Config secrets (any key matching `key|token|secret|password`, case
insensitive) are redacted in the `run_start` record. The transcript is never
truncated, even for large `get_evidence` payloads — an overflowing result is
itself a finding, not something to shorten quietly.

## Switching providers

Each agent's provider, model, and API key source are independent config
values:

```yaml
provider: anthropic       # or: openai
model: claude-opus-4-8    # or any model string that provider's SDK accepts
api_key_env: ANTHROPIC_API_KEY
```

WatchDog on one provider and the executor on another is fully supported —
adding a new provider means adding a class to `agent_harness/providers.py`
and one line in its factory map; nothing in `runner.py` changes.

## What this harness deliberately does not do

- No tool an agent can call that isn't on its own configured servers.
- No retry, repair, or validation logic wrapped around either agent's
  decisions — MCP servers validate their own inputs, and their rejections
  (`{"ok": false, "error": ...}`) are data for the agent to reason about, not
  something the runner intercepts.
- No summarization of tool results.
- No supervisor process that launches both agents together.
- `run_notes` in the task file is always empty, including in fault-injection
  runs where the registry has been deliberately corrupted — neither agent may
  be told that, or the comparison the study rests on is invalidated.
