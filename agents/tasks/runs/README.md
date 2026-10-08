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
| r1 | wrong object (object_id swap) -- RETIRED, files removed: the executor corrected the contradictory object_id back to u_brown, so no fault occurred (its one run was deleted by the operator) | -- | -- |
| r2 | missed grasp | `pick_u_orange`: `grasp_center_offset.position.y` -0.02 -> **-0.045** (was -0.032 at commit 900626f, never run; sha 88f936f1) (place steps keep -0.02) | `{step: pick_u_orange, object: u_orange}` -- slip/drop on lift. If it holds and mis-seats instead, label `{step: place_u_orange, object: u_orange}`. |
| r3 | pressed too hard | **NOT a task variant** -- the file is identical to the committed task; launch with plain `./run_executor.sh 2`. The fault is the install registry's `max_clamp_effort` lowered below the normal grip (see batch.yaml notes) | `{step: pick_<obj>, object: <obj>}` for each object whose limit was lowered |
| r4 | wrong object (geometric; replaces r1) | `pick_u_brown`: `grasp_center_offset.position` (0.025, -0.02, 0) -> **(-0.2396, -0.02, -0.0326)**: u_brown-relative offset that lands on u_orange's normal grasp pose (both parts' start poses are identical across runs; composed via utils/geometry.grasp_center_from_object_and_offset, 0.05 mm error). Everything else still names u_brown, so the executor has no contradiction to correct. | `{step: pick_u_brown, object: u_orange}` (wrong part gripped/moved) and `{step: pick_u_brown, object: u_brown}` (declared part never in hand). Then label what follows physically (u_orange likely carried to u_brown_placement). |

**r4 needs an operator companion.** grasp_skill attaches u_brown (the step's
object_id) in MoveIt while the fingers hold u_orange, which MoveIt still has
as a world object, so lift fails START_STATE_IN_COLLISION (seen on the first r4 attempt,
whose session was deleted by the operator). Before the executor starts, run in a ROS-sourced terminal:
`python3 scripts/r4_allow_finger_contact.py` -- it allows u_orange <-> RG2
link contact in MoveIt's ACM until pick_u_brown ends, logging to
`<session>/operator_acm_override.jsonl`.

Rules:
- Edit only the one step the fault lives in; leave everything else identical
  to the committed task.
- Commit the edited variant BEFORE its first run; the launcher records its
  sha256 in `agent_config_executor.json`, and batch.yaml's session entry
  carries it as `run_task_sha256`.
- Changing a variant after a run makes it a different fault: copy it to a new
  rN instead.
