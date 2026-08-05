# WatchDog / Executor Agent Harness

Two independent agents study the same assembly task from opposite sides:

- **Executor** — commands the robot through the `gripper`, `motion`, and
  `scene` MCP servers. Plans, calls skills, reports outcomes.
- **WatchDog** — an independent supervisory conformance monitor, reachable
  only through the `watchdog` MCP server. Observes the cell, arms
  deterministic monitors, halts on bound violations, records verdicts. It
  never commands the robot.
- **Orchestrator** (`ros2_ws/src/cell_orchestrator/`) — a third, plain ROS
  node with no LLM and no reasoning. It serves the cell's synchronization gate
  (`/cell/request_go`, `/cell/signal_ready`) so the executor blocks between
  announcing a step and acting on it, giving WatchDog a chance to arm before
  anything moves. See "The synchronization gate" below.

The runner (`agent_harness/runner.py`) is identical for both agents and
contains no agent-specific logic. An "agent" here is exactly three things: an
MCP server list, a system prompt, and a task rendering — see `config/` and
`prompts/`. The orchestrator is not an agent in this sense at all — it has no
prompt, no provider, and no configuration about what should be monitored, and
it is not part of this directory: it is its own colcon package, deliberately
split out by dependency type (see "Why they are separated"). This directory
(`agents/`) holds only the two LLM runners, their configs, prompts, and
transcripts — nothing under `agents/` is colcon-built.

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
- **The orchestrator must not serve as WatchDog's mouthpiece into the cell.**
  It only relays a service handshake it does not interpret. If WatchDog served
  `/cell/request_go` directly, the executor could not run at all without
  WatchDog up, and WatchDog would sit literally in the cell's control path —
  indefensible alongside a claim of independent supervision. With the
  orchestrator in between, the cell has an interlock that happens to consult a
  monitor; if the monitor vanishes, the interlock times out and opens rather
  than jamming shut.
- **The orchestrator must not live inside `robot_backend` either.** It is its
  own colcon package, `cell_orchestrator`, whose `package.xml` declares
  `rclpy` and `cell_sync_interfaces` only — it must never declare
  `robot_backend` or `robot_backend_interfaces` (that constraint is stated in
  the package.xml itself, not just here, so it's visible at the build level).
  `robot_backend` is the executor stack; the orchestrator is the third party
  that gates it. A dependency graph showing the executor package containing
  its own supervisor is the wrong reading of that graph, and it costs nothing
  to avoid. This is also why the orchestrator isn't under `agents/`: that
  directory is LLM runners needing `anthropic`/`openai`/`mcp`/`python-dotenv`
  in a venv, with no ROS dependency at all — splitting by dependency type
  keeps colcon builds clean and the venv free of ROS.

`agent_harness/config.py` warns loudly (and refuses to load) if a single
config lists the `watchdog` server alongside any executor server — that
combination would leak monitoring tools to the executor and is treated as a
configuration bug, not a valid setup.

## Setup

1. **Build the workspace, including `cell_sync_interfaces` and
   `cell_orchestrator`.** The gate's interfaces package does not build-depend
   on `robot_backend_interfaces` or `robot_backend`, and the orchestrator
   itself depends on `rclpy` and `cell_sync_interfaces` only (see its
   `package.xml`) — nothing in the executor or WatchDog stacks depends on
   either, on purpose:

   ```bash
   cd ~/watchdog/WatchDog_V1/ros2_ws
   colcon build --packages-select cell_sync_interfaces cell_orchestrator robot_backend_interfaces robot_backend
   ```

2. **Source the ROS overlay** before launching any of the three processes —
   the MCP servers (`gripper_mcp_server`, `motion_mcp_server`,
   `scene_mcp_server`, `watchdog_mcp_server`) are ROS2 nodes, and so is the
   orchestrator (`cell_orchestrator`); all of them need the workspace on the
   Python path and `AMENT_PREFIX_PATH` set up:

   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/watchdog/WatchDog_V1/ros2_ws/install/setup.bash
   ```

3. **Install harness dependencies** (separate from the ROS Python
   environment if you use one) — needed for the two LLM agents, not for the
   orchestrator, which has no LLM dependency at all:

   ```bash
   cd ~/watchdog/WatchDog_V1/agents
   pip install -r requirements.txt
   ```

4. **Copy `.env.example` to `.env` and fill in your keys:**

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
python3 run_agent.py --config config/watchdog_agent.yaml  --task tasks/two_cube_stack.watchdog.yaml --dry-run
python3 run_agent.py --config config/executor_agent.yaml  --task tasks/two_cube_stack.executor.yaml --dry-run
```

Each scenario has two task files, one per role (`<scenario>.executor.yaml`,
`<scenario>.watchdog.yaml`) — always pair the watchdog config with the
`.watchdog.yaml` file and the executor config with the `.executor.yaml` file.
`run_agent.py` exports the role-matching `WATCHDOG_TASK_PATH` /
`EXECUTOR_TASK_PATH` env var (never both) from whichever `--task` path is
given, so a mismatched pairing is a real misconfiguration, not just a naming
slip: get_contract's steps: block would come from the wrong file.

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

## The synchronization gate

Measured agent turn latency runs from low single-digit seconds to nearly a
minute. A monitor that only arms reactively after `begin_step` arms most of a
step late — and for monitors whose reference is captured at arm time, arming
late freezes a reference taken after the scene has already been disturbed. So
the executor blocks between announcing a step (or a mid-step phase) and acting
on it, via a service-based gate rather than the `/watchdog/events` topic: a
dropped release token would be indistinguishable from a slow monitor, and the
topic already has a known discovery-race loss mode (see EventBus hardening
below). Services give a real handshake — `wait_for_service` proves the peer
exists, and every call has an explicit success or failure.

Three processes divide this cleanly:

- **`begin_step` / `mark_phase`** (executor, via `robot_backend.cell.gate`)
  publish their event, then call `/cell/request_go` and block.
- **`signal_ready`** (WatchDog) calls `/cell/signal_ready` once it has armed
  and verified references for the step or phase, releasing the matching
  `request_go` call.
- **The orchestrator** (`cell_orchestrator/node.py`, its own colcon package
  under `ros2_ws/src/`) serves both services and relays between them. It is a
  plain ROS node — no LLM, no reasoning, no knowledge of contracts, monitors,
  claims, or verdicts. See its module docstring for the concurrency
  requirement (`MultiThreadedExecutor` + `ReentrantCallbackGroup`, same
  precedent as `SceneTFSyncNode`) and why it sits in the middle rather than
  folding into WatchDog.

**The gate never refuses.** If the orchestrator is not running, or times out,
or errors, the executor proceeds — `begin_step`/`mark_phase` fail open, and an
unguarded step is a recorded finding, never a deadlock. Executor-alone runs
(no orchestrator, no WatchDog) work unchanged.

## Launch protocol for a run

The orchestrator is cell infrastructure, not agent infrastructure: it comes up
with the cell (`sim_moveit_bringup.launch.py` includes it by default, gated
behind `start_orchestrator:=true`), not with either agent. The operator
sequence is therefore:

```bash
# 1. Bring up the cell — this starts cell_orchestrator too (start_orchestrator
# defaults to true). For an executor-only run, declare that up front:
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py guard_mode:=unguarded
# or leave the default (guard_mode:=guarded) for a monitored run:
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py

# 2. WatchDog — warms its recorder buffers and signals run-scope readiness.
# Also the FIRST of the two agent processes to launch: it writes the shared
# session pointer (~/.watchdog/current_session) that the executor process
# joins in step 3, so both agents' events land in one session directory.
# Skip this step entirely for an executor-only run.
python3 run_agent.py --config config/watchdog_agent.yaml --task tasks/two_cube_stack.watchdog.yaml

# 3. Executor — its first begin_step blocks until WatchDog releases it
# (guarded mode) or returns immediately (unguarded mode). Launched second so
# it joins the session WatchDog already established.
python3 run_agent.py --config config/executor_agent.yaml --task tasks/two_cube_stack.executor.yaml
```

To run the orchestrator standalone instead of through cell bringup (e.g. for
the gate tests below), `ros2 launch cell_orchestrator orchestrator.launch.py
guard_mode:=guarded gate_timeout_sec:=90.0 log_dir:=~/.watchdog/orchestrator`
still works exactly as before.

**Cell (with orchestrator), then WatchDog, then the executor — always**, for a
guarded run. For an executor-only run, skip WatchDog and set
`guard_mode:=unguarded` at launch (or flip it later with `ros2 param set
/cell_orchestrator guard_mode unguarded` — takes effect on the very next gate,
no restart, since the simulator can take minutes to come back up).

Whether a run is guarded is declared, not inferred — a heuristic that detects
"no monitor" could silently misclassify a broken run as an intentionally
unmonitored one, and those two must never be confusable in results. Every gate
response and log record carries one of exactly three reason codes:

- **`unguarded_by_config`** — the operator declared this run unmonitored
  (`guard_mode:=unguarded`). Expected, not a defect. The gate returns in
  milliseconds, never waits.
- **`timeout_gate`** — a monitor was present (it signalled run-scope
  readiness, or something else, at some point) but failed to arm this
  particular step in time. A coverage failure for one step, counted in
  results, not a broken run.
- **`unguarded_latched`** — `guard_mode` was left `guarded` but no monitor
  ever signalled anything at all by the time this run's *first* gate timed
  out. This is a misconfiguration, not a slow step: **leaving `guard_mode`
  guarded without starting a monitor produces a run that completes but
  monitors nothing, and its results cover nothing.** The orchestrator logs
  this once at warning level the moment it latches, then every subsequent
  gate in the run returns `unguarded_latched` immediately (no re-paying the
  timeout — at 90s and two gates per step, that would otherwise cost a
  single-step run three minutes of dead waiting). If a monitor signals at any
  point afterward, the latch clears and normal gating resumes from the next
  request onward.

`released` (gate opened normally) and `error`/`no_orchestrator` (the
`robot_backend.cell.gate` client's own fail-open reasons, used when the
orchestrator process itself isn't reachable at all — a corner case now that
it comes up with the cell, but still a supported one) round out the full set
of reasons a `begin_step`/`mark_phase` call can report.

This is still a manual, three-step operation every time, by design — the
orchestrator does not launch either agent, and neither agent launches the
other; only the orchestrator now shares a lifecycle with the cell itself
rather than needing its own separate terminal.

`--provider` and `--model` CLI flags on `run_agent.py` override the config
file if you need to swap providers for a single run without editing YAML.

## Transcripts

Every run writes a JSONL transcript to `transcripts/` (gitignored, created at
runtime), one record per line, flushed on write so a crashed or halted run
stays readable. Filename pattern:

```text
<agent>_<task_name>_<utc_iso>.jsonl
```

e.g. `watchdog_two_cube_stack_20260720T140512Z.jsonl` and
`executor_two_cube_stack_20260720T140508Z.jsonl` — a paired executor/WatchDog
run over the same task sorts together (same `<task_name>`) and is told apart
by the `<agent>_` prefix. Every record carries `wall_ns` on the same axis as
the cell's own event and monitor-fire timestamps, which is what lets you
correlate an agent's turn timing against physical events after the fact —
and, for WatchDog, is the evidence that a halt never waited on the agent.

This is the agent's record of what it decided and when — a complement to the
cell's own `/watchdog/events` JSONL, not a duplicate of it. `run_start` keeps
only `agent`, `provider`, `model`, `task_name`, and a sha256 of the system
prompt file (so a run can be tied to the prompt version that produced it
without storing the 3.6KB prompt itself, repeated on every turn); it does not
carry `git_commit`, `task_path`, a config echo, or a command line. `run_end`
always carries a corrected `termination_reason` — `provider_error` and
`mcp_error` carry the exception type and message alongside it, and
`max_turns_exhausted` is only ever reported when the turn counter actually
reached the configured `max_turns`, never as a fallback for an unrelated
failure. Any dict key matching `key|token|secret|password` (case insensitive)
is redacted wherever it appears. The transcript is never truncated, even for
large `get_evidence` payloads — an overflowing result is itself a finding, not
something to shorten quietly.

Each turn resends the full, growing conversation to the provider — required
by the stateless completions API, not a bug — but `llm_request` records only
`new_messages`: the messages appended since the previous turn's
`llm_request`, not the whole history. Turn 1's `new_messages` is the initial
task message; turn 2's is whatever was appended after turn 1 (the assistant's
response and any tool result(s)); and so on. To reconstruct the full context
as of any turn N, concatenate `new_messages` from every `llm_request` record
with `turn <= N`, in order.

`llm_response` carries a `reasoning` field — `{"summary": <str or None>,
"tokens": <int or None>}` — the provider's reasoning summary (OpenAI) or
extended-thinking text (Anthropic) for that turn, plus the reasoning token
count from `usage` when the provider reports one. `summary` is `None`
whenever the provider generated no reasoning content for that turn; the API
can withhold reasoning summaries for reasons outside this harness's control
(observed directly: gpt-5 sometimes emits zero summary text on a turn that
goes straight to a tool call with little deliberation, and a non-empty
summary on a turn that reasons through a failure before acting), and this is
a normal outcome, never an error. This is deliberately narrower than the full
provider response: only the extracted summary text and token count are kept,
never the raw payload — restoring `raw` wholesale was considered and
rejected, since the reasoning content was the only part of it worth keeping.

OpenAI reasoning summaries require the Responses API — Chat Completions does
not expose them for reasoning models, so `OpenAIProvider` uses
`client.responses.create`, not `client.chat.completions.create`. Reasoning
continuity across turns depends on echoing the model's own prior reasoning
output items back on the next call (OpenAI's own guidance, not just this
harness's convention) and Anthropic's extended thinking has an equivalent
requirement for thinking blocks preceding a tool-use turn — losing either
degrades reasoning quality silently, it does not raise. Both providers carry
this forward via `provider_native` on the assistant message, an opaque field
`runner.py` copies through without reading; only the originating provider's
own message-conversion method interprets it.

## Switching providers

Each agent's provider, model, and API key source are independent config
values:

```yaml
provider: anthropic       # or: openai
model: claude-opus-4-8    # or any model string that provider's SDK accepts
api_key_env: ANTHROPIC_API_KEY
reasoning_effort: medium          # openai only: minimal | low | medium | high
thinking_budget_tokens: 4096      # anthropic only; part of max_tokens, not additional to it
```

Both reasoning fields are always present in config and validated at load
time regardless of which provider the config targets — `reasoning_effort` is
ignored by `AnthropicProvider`, `thinking_budget_tokens` by `OpenAIProvider`
— so switching `provider:` doesn't require editing them out. WatchDog on one
provider and the executor on another is fully supported — adding a new
provider means adding a class to `agent_harness/providers.py` and one line
in its factory map; nothing in `runner.py` changes.

## Skill event coverage

Every executor MCP tool that changes or queries the physical cell or the
planning scene emits a `skill_start`/`skill_end` pair on `/watchdog/events`
(directly via `@emit_skill_events`, or by delegating to an internally
decorated skill function — `grasp`/`release`/`open_gripper`/etc.), or emits
its own dedicated event type (`begin_step`/`end_step`/`mark_phase`). Only
pure local computation that touches nothing (`compute_grasp_approach_poses`,
pose geometry only) is exempt. `ros2_ws/src/robot_backend/test/
test_skill_event_coverage.py` enumerates every tool actually registered on
each server's FastMCP instance and fails if any tool is neither decorated nor
in its allowlist (which documents *why* each exemption is safe) — this is
what stops a newly added tool from silently reopening a coverage gap. Run it
whenever a tool is added, renamed, or its call path changes.

## EventBus hardening

Every executor MCP server (`scene`, `motion`, `gripper`) and the WatchDog
server construct their backend/node/event bus **eagerly in `main()`**, before
`mcp.run()`, rather than waiting for the first tool call. The runner spawns
all server subprocesses roughly 40s before the first LLM turn lands (that
turn is what triggers the first real tool call), so this lets DDS discovery
of `/watchdog/events` complete during that idle window instead of paying the
bounded subscriber wait below on the first real publish — and it means
WatchDog's recorder has been buffering TF/joint state since process start, so
its first `signal_ready(scope="run")` warm check passes on the first try
instead of needing a retry. Eager init is non-fatal: if it fails (simulator
not up, ROS overlay not sourced), each server logs a warning and falls
through to `mcp.run()` unchanged — the lazy `_ensure_backend()` /
`_ensure_watchdog()` path inside each tool remains the fallback, and
`--dry-run` keeps working with no simulator running at all.

Separately, `robot_backend.events.bus.EventBus` waits, once per process and
only before its first publish, for a matched subscriber on
`/watchdog/events` (bounded by `WATCHDOG_EVENTBUS_WAIT_SEC`, default 2.0s) —
this covers the DDS discovery race that can silently drop a cold-started
publisher's very first message (always `begin_step`, for the executor's
servers) under default volatile QoS.
It logs once at warning level and publishes anyway if the wait times out; it
never raises and never blocks on any publish after the first. Every event
envelope also carries `subscriber_count` (captured at publish time — an event
recorded with `subscriber_count: 0` is a provable drop) and a per-process
monotonic `seq` starting at 1 (a gap in `seq`, unlike a UUID `event_id`, is
detectable — reconciling per-process JSONL logs against what a topic
subscriber actually received is mechanical). `TRANSIENT_LOCAL` durability was
deliberately not used to fix the discovery race: it would also replay
buffered `begin_step` events to a later-joining subscriber, which could arm
monitors for steps that have already finished — replay is more dangerous here
than loss.

Reconcile a paired run's per-process JSONL logs against what actually arrived
on the topic with `ros2_ws/src/robot_backend/scripts/reconcile_event_seq.py`
(`capture` subcommand records the topic; `check` compares one or more
per-process logs against that capture and reports any `seq` gap by name).
This should be a standard post-run check for every evaluation run.

## MCP client timeout vs. gate timeout

`begin_step`/`mark_phase` can block for as long as the monitoring agent takes
to arm — up to the gate's own timeout at the orchestrator. If this harness's
own MCP client timeout were shorter than (or merely equal to) that gate
timeout, the client would abort the call while the orchestrator is still
legitimately holding it open. The symptom is an `mcp_error` in the executor's
transcript with the orchestrator log showing a perfectly ordinary `gate_open`
a few seconds later — that looks exactly like a transport bug and is actually
a configuration mismatch, and it would cost hours to find as anything else.

Required ordering: **`mcp_client_timeout_sec` > `gate_timeout_sec` > expected
agent arming turn.** Both values are explicit, required config fields (see
`executor_agent.yaml`), and `agent_harness/config.py` enforces the first
inequality at load time — a config where the MCP client timeout is at or
below the gate timeout fails to load at all, loudly, rather than failing
mysteriously at runtime. It also cross-checks the executor's `scene` server's
own `CELL_GATE_TIMEOUT_SEC` env var (what `robot_backend.cell.gate` actually
uses as its default `timeout_sec`) against the config's `gate_timeout_sec`,
so the two independently-run processes can't silently drift apart even when
each value looks fine on its own. WatchDog's config carries
`mcp_client_timeout_sec` too (sized against its own longest blocking call,
`wait_for_event`) but no `gate_timeout_sec` — it never calls `request_go`.

## Verification

Each of these is a gate — do them in order before trusting a full run:

1. **EventBus, no agents.** Start the stack, `ros2 topic echo
   /watchdog/events`, make one tool call by hand. The first event must show
   `seq: 1` and a non-zero `subscriber_count`. Kill the echo, make another
   call, restart the echo, make a third — the gap in `seq` should make the
   missed event obvious.
   - **1b, JSONL stamping.** After any tool call, `cat` the fresh
     `events_*.jsonl` (e.g. `events_watchdog.jsonl`) this process wrote and
     confirm `seq` and `subscriber_count` are present on *every* line, not
     just on what the topic delivered — the reconciliation script (see "MCP
     client timeout vs. gate timeout" above) compares these two, and the
     comparison is dead if the per-process log never carried the field to
     begin with.
2. **Runner termination.** Force a provider failure with an invalid model
   name; `run_end` must say `provider_error`. Then run with `max_turns: 3` and
   confirm a real cap reports `max_turns_exhausted`.
3. **Guard mode.** (Replaces the old "gate fail-open" check, which expected
   `reason: "no_orchestrator"` — that was only correct back when the
   orchestrator had to be started by hand and a run without it was the normal
   executor-alone case. Now that the orchestrator comes up with cell bringup,
   the primary behavior to verify is guard_mode, not orchestrator absence.)
   Orchestrator alone (`ros2 launch cell_orchestrator orchestrator.launch.py`)
   for all four sub-checks:
   - **3a, unguarded by config.** `guard_mode:=unguarded`. Call
     `/cell/request_go` — must return in milliseconds with `proceed: True`,
     `timed_out: False`, `reason: "unguarded_by_config"`.
   - **3b, guarded with no monitor latches.** `guard_mode:=guarded`, no
     `signal_ready` call ever made. The *first* `/cell/request_go` must block
     for the full `gate_timeout_sec` before returning `reason:
     "unguarded_latched"`, and the orchestrator must log a warning-level
     message at that moment. Every subsequent `/cell/request_go` call (any
     `step_id`) must then return immediately (milliseconds) with the same
     `reason: "unguarded_latched"` — confirm it does *not* re-pay the
     timeout.
   - **3c, guarded with a monitor.** `guard_mode:=guarded`. Call
     `/cell/signal_ready` (scope `run`, then scope `step` for the step you're
     about to request) before calling `/cell/request_go` — the gate must
     return `reason: "released"`. Confirm `unguarded_by_config` never appears
     anywhere in a run where a monitor actually signalled.
   - **3d, dynamic reconfiguration.** With the node still up from 3b or 3c,
     `ros2 param set /cell_orchestrator guard_mode unguarded` — the *next*
     `/cell/request_go` call must immediately return `unguarded_by_config`,
     with no node restart.
4. **Service concurrency.** Orchestrator alone. Call `/cell/request_go` from
   the command line, and while it blocks, call `/cell/signal_ready` from a
   second terminal. The first call must return. If it hangs, the callback
   group or executor is wrong — test this before involving any agent.
   - **4b, deadline sharing.** Orchestrator alone, `guard_mode:=guarded`.
     Call `/cell/request_go` with `timeout_sec: 5.0` and never signal
     anything — the call must return in ~5s (not 10), with `reason:
     "unguarded_latched"` and `proceed: True` (this is 3b's timing bound
     restated: exactly one full timeout, not two). Then signal run + step
     readiness, call again with a *different* `step_id`, and confirm ~5s with
     `reason: "timeout_gate"` if you withhold that step's own signal.
   - **4c, timeout ordering.** Confirm `agent_harness/config.py` rejects a
     config where `mcp_client_timeout_sec` is at or below `gate_timeout_sec`
     (e.g. set both to the same value in `executor_agent.yaml` and try to
     load it) — it must raise at config load, before any process starts.
5. **Gate closed-loop.** Cell bringup (orchestrator included, default
   `guard_mode:=guarded`), then WatchDog, then executor. `begin_step` must
   block, WatchDog must arm and signal, and `begin_step` must return with
   `waited_sec` in the seconds. Confirm from the orchestrator's own JSONL
   that the ordering was `orchestrator_start` (with `guard_mode: "guarded"`),
   `run_ready`, `gate_open` — not `gate_unguarded_latched` or `gate_timeout`.
6. **A full step end to end** — an `end_step` with `claimed_outcome` and a
   populated `claim_detail`.

F2 needs a real provider call and F6 needs the full Isaac Sim + MoveIt stack
— both are on the operator to run; everything else here can and should be
checked before either agent ever touches the real cell.

## What this harness deliberately does not do

- No tool an agent can call that isn't on its own configured servers.
- No retry, repair, or validation logic wrapped around either agent's
  decisions — MCP servers validate their own inputs, and their rejections
  (`{"ok": false, "error": ...}`) are data for the agent to reason about, not
  something the runner intercepts.
- No summarization of tool results.
- No supervisor process that launches both agents together, or that launches
  the orchestrator together with either agent.
- The orchestrator never reads contracts, monitor state, claims, or verdicts,
  and never inspects event payloads beyond what the gate's own service
  request/response carries. Its `RequestGo` response carries nothing the
  executor did not already send it — timing coupling, not information
  coupling.
- The gate never refuses under any condition; `proceed` is always `True`.
- `TRANSIENT_LOCAL` durability on `/watchdog/events` — see EventBus hardening
  above.
- `run_notes` in the task file is always empty, including in fault-injection
  runs where the registry has been deliberately corrupted — neither agent may
  be told that, or the comparison the study rests on is invalidated.
