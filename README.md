# WatchDog_V1

A simulated robot assembly cell in which two independent LLM agents work the
same task from opposite sides: an **executor** that drives a UR5e arm through
a pick-and-place assembly, and a **WatchDog** that supervises it without ever
touching the robot. The repository holds the cell (ROS 2 + MoveIt + Isaac
Sim), the agents, the fault-injection and scene-event tooling, the scorers,
and every scored run behind the paper's numbers.

The research question: can an LLM agent act as an independent conformance
monitor for another LLM agent's robot work, and how does that compare with the
executor monitoring itself?

## The study in one paragraph

The executor assembles four FMB parts onto a fixture (`fmb_assembly_2`,
12 steps). Four **scenarios** vary what the supervisor can do:

| Scenario | Who monitors | Monitor surface |
|---|---|---|
| S1 | WatchDog | library monitors only (`arm_monitor`) |
| S2 | WatchDog | library + agent-authored Python monitors (`arm_custom_monitor`) |
| S3 | WatchDog | agent-authored monitors only |
| S4 | the executor itself | agent-authored monitors, no WatchDog session |

Four **classes** vary what goes wrong: C1 clean, C2 an injected or natural
fault (slip, mis-seat, wrong part, displaced bystander), C3 a benign plan
deviation, C4 a corrupted specification handed to the executor. Every run
also injects **scene events** the monitor must notice: a foreign cube placed
on the bench and, in later batches, a hand passing through the workspace.

Two studies were run on this design. The **baseline** uses Claude Sonnet 5 at
low effort for both agents across S1 to S4 (23 runs each). The **capability
study** reruns S3 and S4 with Claude Opus 5.5 at low and medium effort. A
scripted **reference watchdog** (no LLM) provides the upper bound for scene
observation. Results live in `paper_results/`.

## How a run works

```
 Isaac Sim ──/tf, /joint_states, camera──▶ ROS 2 cell (MoveIt, orchestrator, event bus)
                                              │                      │
                               command MCP servers            watchdog MCP server
                            (motion, gripper, scene)       (events, evidence, monitors)
                                              │                      │
                                    Executor agent              WatchDog agent
                                   (Claude Code CLI)           (Claude Code CLI)
```

- The **cell** (`ros2_ws/`) exposes the robot to the agents only through MCP
  servers. The executor's servers can move the arm; the WatchDog's server can
  only observe, arm deterministic monitors, and record verdicts.
- The **orchestrator** is a plain ROS node with a gate: the executor blocks
  at every step and phase boundary until the WatchDog signals ready, so the
  WatchDog can arm before anything moves. In S4 the gate runs unguarded.
- The **WatchDog** reads TF, joint states and the camera, arms monitors with
  thresholds that must trace to the part registry, is woken when one fires,
  looks at the camera when it needs to, and records step verdicts plus one
  final run verdict.
- Everything each agent did lands in one **session directory** under
  `agents/transcripts/sessions/<timestamp>/`: event logs, MCP call logs, the
  rosbag, camera frames, authored monitor sources, injection ground truth,
  and the operator's hand-written label. Session directories are never
  committed.
- **Scorers** turn a session plus its label into one row per scorer, and
  `paper_results/batches/<batch>/` collects those rows with full provenance.

## Repository map

| Path | What it is |
|---|---|
| `ros2_ws/` | The ROS 2 Jazzy workspace: `robot_backend` (skills, MCP servers, monitor engine, recorder, vision), `cell_orchestrator` (the gate), `sim_moveit_bringup` (launch), `isaac_trajectory_bridge`, robot and gripper descriptions. |
| `ros2_ws/src/robot_backend/config/model_registry.yaml` | The part registry: every object's tolerances and limits. Monitor thresholds must trace to it. |
| `agents/` | The two agents: prompts, task files per role and scenario, configs, the API-path runner, the harness tests. See `agents/README.md`. |
| `agents/tasks/runs/` | Class 2 and 3 task variants (`rN`), one fault each. See its README. |
| `isaac/script_editor/` | Operator tools pasted into Isaac's Script Editor: intruder placement, visitor and nudge scenarios, the pinned per-batch injection scripts. |
| `scripts/` | Operator and scoring helpers: preflight check, leak audit, corrupted-registry builder, Class 2 companions. |
| `watchdog_scorer.py`, `s4_scorer.py`, `observation_scorer.py` | The three scorers. See `docs/dev/SCORERS.md`. |
| `label_templates/` | Ground-truth label templates copied into a session directory after each run. |
| `paper_results/` | Scored batches: `batch.yaml` provenance, `raw/` scorer output, `corrected/` with a corrections log, `summary/`. See its README. |
| `reference_watchdog/` | The scripted oracle watchdog and its reference monitors, with offline tests. See its README. |
| `run_watchdog.sh`, `run_executor.sh` | Launchers for the two Claude Code sessions. |
| `*.mcp.json`, `*.settings.json` | Per-role MCP configs (one per scenario, assembly and variant) and per-role model, effort and permission settings. |
| `docs/dev/` | Procedures and design records. Start with `docs/README.md`. |
| `assets/`, `config/`, `generated_urdf/` | USD stage, robot and gripper assets, UR5e configs. |

## Prerequisites

- Ubuntu with **ROS 2 Jazzy** and **MoveIt 2**, system Python 3.12.
- **Isaac Sim** with the cell stage `assets/usd/assembly_room.usd`.
- The **Claude Code CLI** on PATH (standalone, not the VS Code extension
  binary). Scored runs are driven through it; no API key is needed.
- Two Python virtual environments, kept separate on purpose:
  - `ros2_ws/.venv` for the MCP servers and scorers: `mcp<2`, `numpy<2`,
    `trimesh`, `PyYAML`, `pytest` (`ros2_ws/requirements.txt`). numpy must
    stay below 2 or every mesh load fails.
  - `agents/.venv` for the API-path runner only (`agents/requirements.txt`).
    Never activate it in a shell that launches the servers: they re-exec into
    whichever venv is active, and `mcp` 2.x breaks them.

## Setup

```bash
# 1. workspace venv
python3 -m venv ros2_ws/.venv
ros2_ws/.venv/bin/pip install -r ros2_ws/requirements.txt

# 2. build the cell (system Python is pinned by ros2_ws/colcon_defaults.yaml)
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
cd ..

# 3. tests (never system pytest, never colcon test)
cd ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/src/robot_backend:$PYTHONPATH" \
  .venv/bin/python3 -m pytest -q src/robot_backend/test
cd ..
```

## Running one scored run

The complete, ordered procedure is `docs/dev/BATCH_RUNBOOK.md`; the
mechanisms behind each step are in `docs/dev/RUN.md`. In outline:

1. **Isaac Sim**: load the stage, reset simulation time, press play. Paste
   the Script Editor preamble from `isaac/script_editor/inject_scenarios.py`.
2. **Bringup**, in a fresh shell with nothing exported:
   `ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py`
   (add `guard_mode:=unguarded` for S4). This creates the session directory
   and writes `~/.watchdog/current_session`.
3. **WatchDog session** (S1 to S3 only): `./run_watchdog.sh s3 2`. Wait until
   it has armed and signalled run-scope readiness.
4. **Executor session**: `./run_executor.sh 2`, or `./run_executor.sh s4 2`
   for S4, or `EXECUTOR_VARIANT=r2 ./run_executor.sh 2` for a Class 2 variant.
5. **Inject** the batch's scene events from the Script Editor
   (`isaac/script_editor/s1_c1_inject.py` is the pinned cube plus hand).
6. When the run ends, **Ctrl-C the bringup** to finalize the rosbag.
7. **Label** the run: copy a template from `label_templates/` into the
   session directory and fill it in from what physically happened.
8. **Score** it into the batch's `raw/`, then register it in `batch.yaml`.

Model and effort for each role are set at the top of
`watchdog.settings.json` and `executor.settings.json` and are echoed at
launch. A scored run must use `--strict-mcp-config`, which the launchers pass,
so each session sees only its own servers.

## Reading the results

Each batch under `paper_results/batches/` has a `batch.yaml` that records the
commit, registry hash, models, injection design, every session with notes,
and any excluded sessions. `raw/` is append-only scorer output; `corrected/`
is the hand-corrected copy with every edit logged in `CORRECTIONS.md`;
`summary/summary.txt` is the per-scenario rollup. Paper numbers come from
`corrected/` only.

Batches: `S1` to `S4` (Sonnet 5 low), `S3_opus_low`, `S3_opus_medium`,
`S4_opus_medium` (Opus 5.5), `REF` (scripted oracle), and `20261005_smoke`
(pipeline check, never scored).

## Where to go next

- `docs/README.md` for the document index, including which records are
  historical.
- `agents/README.md` for the agent harness, the gate, and why the two agents
  are isolated from each other.
- `docs/dev/SCORERS.md` for what every scorer column means and the
  hand-correction rules applied at Phase 6.
- `reference_watchdog/README.md` for the oracle watchdog and the lessons from
  getting a depth-based scene monitor to work in this cell.
