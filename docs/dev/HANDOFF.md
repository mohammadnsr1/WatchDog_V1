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

Both halves are now required; omitting the orientation raises. Identity is not
a safe default: it aligns the grasp with the object's own frame, which for a
part resting square points the gripper UP and inverts the approach axis.

**One offset per held span.** Because the offset IS the object→gripper
transform, and that transform is frozen between a grasp and its release, a
single value must hold for every step in that span (pick → lift → approach →
place). If a step mid-span wants a different orientation, something is wrong:
a rigid grasp cannot change without a release. This is a cheap invariant to
check and it caught two real authoring errors.

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
| --- | --- | --- |
| `target_offset_x/y/z/q*` | `move_grasp_center_to_pose` | composed in frame axes, in-tool |
| `max_velocity/acceleration_scaling_factor`, `allowed_planning_time` | `move_grasp_center_to_pose` | were hardcoded; needed for slow seating |
| `approach_distance_m` | `move_grasp_center_to_pose` | **lifts**: offsets the resolved grasp centre along its own local Z |
| `clear_octomap_first` | both move tools | a separate clear refills before the plan runs |
| `retreat_tcp`, `clear_octomap` | new tools | |

**Lifts go along the approach axis, not world Z.** `approach_distance_m` on the
move tool offsets the *grasp centre* along its own local Z — the same axis the
approach came in on, since the grasp orientation is unchanged. Sign convention
is the SAME as the pick's (negative = back away); it is not the pick's
opposite. A `target_offset` lift is in the OBJECT's axes and only goes up while
the part rests square — for a part standing at a reorient frame it shoves it
200 mm sideways across the bench, which is the "dragging" symptom.

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

## 4b. Images

- Captures are **PNG only**. `utils/png_encode.py` is a dependency-free encoder
  (stdlib `zlib`/`struct`) — the MCP venv has neither OpenCV nor Pillow. Raw
  frames used to be written as `.raw`, i.e. unopenable.
- BGR sources are channel-swapped so they are not colour-inverted. A frame that
  cannot become a real PNG (JPEG) is **skipped and counted**, never written
  under a `.png` name that will not open.
- `image_buffer` now prefers the **raw** camera topic over compressed, because
  PNG encoding needs raw pixels. Raw costs more ring-buffer memory, so fewer
  frames are retained.
- Rosbag is untouched — still a rosbag.
- Only the **watchdog** has a *capture* tool (`capture_frames`), and it remains
  record-only: it preserves a window of the recent past to disk and returns
  paths, not pixels.
- **Both roles can now SEE, via `look()`** (added 2026-09-30, Stage 1 of scene
  observation). It returns the latest frame to the model as real MCP image
  content — `rgb` and/or `depth` — and is on the watchdog server plus
  `robot-backend-scene` for the executor. One shared implementation,
  `robot_backend/vision/look.py`. Full-resolution PNGs and the raw depth `.npy`
  still land in `<session_dir>/frames/`.
  - This supersedes the note that used to sit here about the removed executor-side
    `capture_image`. That tool was dropped because the agent could save a PNG but
    never see it; the fix turned out not to be a `providers.py` change but a
    change of harness. The **Claude Code CLI renders image content correctly**,
    and it is already the sanctioned launch path for scored runs.
  - `agents/agent_harness/providers.py` (~429–432) still stringifies every tool
    result, so under that harness images arrive as base64 text. **Do not patch
    `providers.py`** — treat vision as unavailable there instead. See `RUN.md`
    § Vision.
- Isaac now publishes `/camera/depth/image_raw` (32FC1 metres) and
  `/camera/depth/camera_info`; the point cloud is **no longer published** and
  nothing subscribes to it (`use_octomap` and `filter_pointcloud` both default
  false). So depth is dense and per-pixel, and the point-cloud projection
  fallback in `image_buffer.py` is dormant, kept as an option and unexercised.

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

- **hex resting orientation is unresolved** and everything hex hangs on it. The
  authored pick offset `(0.7071,0,0,0.7071)` is only a top-down grasp if the
  pins rest at `-90,0,180`; at identity or `0,0,180` it is a side grasp, the
  lift drags 200 mm sideways, and the pick/place offsets are inconsistent
  across the span. Get `tf2_echo world hex_blue` before running the hex steps.
- **The agent skips the reorient approach step.** `reorient_approach_*` and
  `reorient_*` share `target_frame`, `object_id` and `grasp_center_offset`,
  differing only by `target_offset` — so they read as two moves to the same
  place. Check the transcript for whether it announced both and merged the
  motion, or never announced the approach at all; the fix differs.
- `target_frame`, `action_hint` are still **prose** — nothing enforces them.
  `standoff_m` and `lift_height_m` had the same problem and became
  `target_offset_*` and `approach_distance_m` respectively.
- Speed knobs are only on `move_grasp_center_to_pose`; the other movers hardcode.
- The executor **cannot call `get_contract`** (watchdog-only server, reads
  `WATCHDOG_TASK_PATH`) though `executor_system.txt` step 2 tells it to.
- `loop.max_turns: 60` against a 24-step task.
- The IDE-side MCP servers (`~/.cursor/mcp.json`, `~/.claude.json`) were
  disabled — they leaked one orphaned ROS node set per reconnect. Backups are
  timestamped alongside each file. `run_agent.py`'s own servers are unaffected.
