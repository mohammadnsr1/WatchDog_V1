# WatchDog_V1 — handoff since plan_assembly_step.py was written

You wrote `plan_assembly_step.py` as a **standalone** validation script whose
conventions deliberately diverged from the live agent stack. That is no longer
true: its conventions were promoted into the shared motion layer, and the agent
now runs the same assembly. This is what changed.

## 1. The conventions (these caused most of the bugs — know them cold)

**`grasp_center_offset` is OBJECT-RELATIVE in both halves**, composed through
the object's live pose:
- `position` is rotated through the object's orientation, so one value names
  the same physical point however the part lies.
- `orientation` is the grasp centre's rotation **relative to the object**, so
  the grasp inherits the part's actual roll/pitch/yaw. This is what lets a part
  re-picked after settling off-square be gripped square to itself.

Read authored values against how the part RESTS at that step. Resting square
with world, `{1,0,0,0}` is top-down and reads that way. Resting rotated, the
value is `inverse(resting) ⊗ desired_world` and will not look like a world
orientation. An earlier revision read orientation as a desired WORLD value
taken verbatim; that made `{1,0,0,0}` literally mean top-down but left a
settled part ungrippable.

Both halves are now required; omitting the orientation raises. It used to
default to identity, which under this convention means "align to world axes"
and inverted the approach axis — pre-grasp staged 14 cm *below* the table.

**Both are resolved by one shared helper**, `utils/geometry.grasp_center_from_object_and_offset`,
called by `motion_mcp_server._compose_grasp_center_from_object` and by the plan
script. The old divergence (MCP composed orientation against the object frame)
agreed only when the object sat at identity — exactly the reorient case it broke.

**`approach_distance_m` is SIGNED** and applied exactly as given along the grasp
centre's own local Z. It is no longer forced negative. For the top-down grasps
here local +Z points *into* the object, so backing off is negative. The MCP
default flipped to `-0.05` to match.

**A pose argument's meaning changes with the offset.** When a
`grasp_center_offset` is supplied, `x…qw` is the **object's** pose (or the
destination's), not the grasp pose. The old docs said "grasp center" and told
callers to pass `(1,0,0,0)` — the agent followed that literally, putting the
grasp orientation in the object slot, which flipped the offset 180° about X and
landed the grasp 80 mm off, on the wrong feature, with no error.

**An attached part's orientation is not free.** Once grasped, the object→gripper
transform is fixed, so the destination orientation determines the required
gripper orientation: `object_target ⊗ (object→gripper at pick)`. An inconsistent
pair plans fine and puts the part somewhere else.

## 2. Plan-script additions

- `frame: target_object` can take **`target_id`** (a live TF frame) instead of a
  literal `object_pose`, plus optional **`target_offset`** composed in the
  **frame's own axes**. These frames' local +Z generally does NOT point up — a
  world-frame add stages the approach sideways (113 mm error, measured).
- **`retreat`** action: a pure TCP move. Not grasp-centre — a release opens the
  jaws, and the grasp-centre→TCP offset is width-dependent, so a grasp-centre
  retreat drives the TCP ~41 mm *down* into the part just placed.
- **`clear_octomap`** action.
- Planner fallback chain, `solve_ik_first`, and per-step tolerances via `motion:`.

## 3. Motion MCP additions (so the agent passes values instead of computing them)

| argument | tool | why |
|---|---|---|
| `target_offset_x/y/z/q*` | `move_grasp_center_to_pose` | composed in frame axes, in-tool |
| `max_velocity/acceleration_scaling_factor`, `allowed_planning_time` | `move_grasp_center_to_pose` | were hardcoded; needed for slow seating |
| `clear_octomap_first` | both move tools | a separate clear refills before the plan runs |
| `retreat_tcp`, `clear_octomap` | new tools | |

Planner: fallback chain 4→2, budget 10 s×10 → 5 s×4, `TimeoutError` folded into
the chain, and `START_STATE_IN_COLLISION` now appends the colliding bodies from
`/check_state_validity`.

## 4. Scene / cell

- Registry gained 6 **pose-only** entries (`*_placement`, `*_reorient`,
  `hex_*_hole`) with `enabled: false` — skipped by collision sync, still
  resolvable by `get_object_pose`. The 1 mm box only satisfies the loader.
- **Octomap and the point-cloud filter are OFF** (`use_octomap:=false`,
  `filter_pointcloud:=false`); nothing publishes a cloud. The bench is a static
  collision slab, `workspace_floor`, published by the bringup
  (`x[-0.9,1.5] y[-0.9,0.9]`, top at z=-0.05). The octomap-clearing guidance in
  the task/prompt is therefore dormant, not wrong — it applies if perception
  returns.

## 5. Environment gotchas

- `robot_backend` is a **copy-install** package. Source edits need
  `colcon build`; registry and launch changes also need the **bringup restarted**
  (`scene_tf_sync_node` reads the registry at startup).
- `test/plan_assembly_step.py` and the `*.plan.yaml` files are **not** installed —
  run them from source, no rebuild needed.
- MCP servers exec into a venv; set `_ROBOT_BACKEND_MCP_VENV_READY=1` to import
  them in-process.

## 6. Known-broken tests — leave them alone (deliberate)

15 failures, none from this work: 6 event-coverage (`FastMCP` not importable in
system python), 6 contract/registry (assert cube values changed in an earlier
commit), 1 `test_grasp_skill` collection error (`AIR_CLOSE_FLOOR_EFFORT` no
longer exists), 1 rg2 width round-trip at 0.0, 2 repo-wide lint.
`test_motion_geometry.py` (10 tests, covering every convention above) passes.

## 7. Open items

- `lift_height_m`, `target_frame`, `action_hint` are still **prose** — nothing
  enforces them, so the agent can substitute values (it once used `-0.08` for an
  authored `-0.10`). `standoff_m` had the same problem and became `target_offset_*`.
- Speed knobs are only on `move_grasp_center_to_pose`; the other movers hardcode.
- The executor **cannot call `get_contract`** (watchdog-only server, reads
  `WATCHDOG_TASK_PATH`) though `executor_system.txt` step 2 tells it to.
- `loop.max_turns: 60` against a 24-step task.
- Prompt updates are the user's stated next task.
