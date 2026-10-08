# Executor task variants (Class 2 faults)

Copies of `../fmb_assembly_2.executor.yaml`, each to be edited to carry ONE
fault, launched with `EXECUTOR_VARIANT=<rN> ./run_executor.sh 2` (which picks
`executor_2_<rN>.mcp.json`). The WatchDog's task file is never varied.

**Names are deliberately neutral.** `get_rendered_task` shows the executor the
resolved path of its task file (role_context.py, `sources.task_path`), so a
path naming the fault would tell the executor what is wrong. YAML comments are
NOT shown (the file is parsed), but keep the fault out of comments anyway, and
never put this mapping anywhere under a name the executor sees.

| variant | fault | edit (fill in) | violation_steps (decide BEFORE the run) |
|---|---|---|---|
| r1 | wrong object | `pick_u_brown`: `object_id` u_brown -> **u_orange** (step fields, incl. grasp offsets, still u_brown's) | `{step: pick_u_brown, object: u_orange}` -- the wrong part is gripped/moved. Add what else physically happens (e.g. u_brown never placed). |
| r2 | missed grasp | `pick_u_orange`: `grasp_center_offset.position.y` -0.02 -> **-0.032** (place steps keep -0.02) | `{step: pick_u_orange, object: u_orange}` -- slip/drop on lift. If it holds and mis-seats instead, label `{step: place_u_orange, object: u_orange}`. |
| r3 | pressed too hard | **NOT a task variant** -- the file is identical to the committed task; launch with plain `./run_executor.sh 2`. The fault is the install registry's `max_clamp_effort` lowered below the normal grip (see batch.yaml notes) | `{step: pick_<obj>, object: <obj>}` for each object whose limit was lowered |

Rules:
- Edit only the one step the fault lives in; leave everything else identical
  to the committed task.
- Commit the edited variant BEFORE its first run; the launcher records its
  sha256 in `agent_config_executor.json`, and batch.yaml's session entry
  carries it as `run_task_sha256`.
- Changing a variant after a run makes it a different fault: copy it to a new
  rN instead.
