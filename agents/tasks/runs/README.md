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
| r1 | wrong object (swap object_id in one pick step) | | |
| r2 | missed grasp (shift grasp_center_offset in one pick step) | | |
| r3 | pressed too hard (narrower grasp_width_m in one pick step) | | |

Rules:
- Edit only the one step the fault lives in; leave everything else identical
  to the committed task.
- Commit the edited variant BEFORE its first run; the launcher records its
  sha256 in `agent_config_executor.json`, and batch.yaml's session entry
  carries it as `run_task_sha256`.
- Changing a variant after a run makes it a different fault: copy it to a new
  rN instead.
