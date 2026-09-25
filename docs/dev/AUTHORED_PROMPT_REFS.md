# Prompt references to the library monitor surface

Every line in the watchdog prompts and watchdog task files that names
`arm_monitor`, `available_monitor_definitions`, or a library monitor type
(`relative_frame`, `stationarity`, `region`, `effort_event`), found by exact
text search. These are the lines a future S3 prompt variant (where the library
surface is not served) would have to account for. No file listed here was
edited.

Search scope: `agents/prompts/watchdog_system.txt`,
`agents/prompts/watchdog_task.txt` (the watchdog task template), and every
`agents/tasks/*.watchdog.yaml`.

## agents/prompts/watchdog_system.txt

| Line | Text |
|---|---|
| 75 | `available_monitor_definitions()` |
| 117 | `arm_monitor(monitor_type, params, clause, monitor_id)` |
| 130 | `disarm_monitor(monitor_id) / disarm_all_monitors()` |
| 353 | `At the start of the run, call available_monitor_definitions and` |

Context: lines 75–84 are the `available_monitor_definitions` entry in the "The
tools" section; lines 117–129 are the `arm_monitor` entry (its text also
explains reference capture and re-arm semantics); line 130 heads the disarm
entry (listed because it names the arm/disarm tool family, though neither
gated tool by name); lines 353–356 are the control-loop instruction to call
`available_monitor_definitions` and `available_event_types` once at run start,
before `signal_ready(scope="run")`.

## agents/prompts/watchdog_task.txt

No matches.

## agents/tasks/fmb_assembly.watchdog.yaml

| Line | Text |
|---|---|
| 218 | `  stationarity requirement is the bystanders' only protection. If a bystander` |

Context: prose inside the bystander-disturbance explanation; "stationarity"
here names the requirement, in the same sentence that assigns catching a
bystander disturbance to the watchdog.

## agents/tasks/fmb_assembly_2.watchdog.yaml

| Line | Text |
|---|---|
| 208 | `  stationarity requirement is the bystanders' only protection. If a bystander` |

Context: same sentence as fmb_assembly.watchdog.yaml line 218.

## agents/tasks/two_cube_stack.watchdog.yaml

No matches.

## agents/tasks/u_green_pick.watchdog.yaml

No matches.

## agents/tasks/single_step_pick.watchdog.yaml

No matches.

## Notes

- No watchdog prompt or task file names `relative_frame`, `region`, or
  `effort_event` literally. The primitive catalog reaches the agent only
  through the `available_monitor_definitions` tool (the system prompt at line
  78 says the prompt "deliberately does not restate it").
- The executor prompts/tasks were out of scope for this search, per the spec.
