# Stage 3 — scene observation: record_observation, intruder injection, leak audit (2026-09-30)

Everything ran with `ros2_ws/.venv/bin/python3` under `unset VIRTUAL_ENV`, ROS
Jazzy + install space sourced, `_ROBOT_BACKEND_MCP_VENV_READY=1`,
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, `PYTHONPATH` prefixed with
`src/robot_backend`. No colcon, no servers, no Isaac was started; the Isaac
script is verified by reading plus unit tests of its pure core.

## 0. Read-first findings

### 0.1 How Isaac publishes object TF today

The object TF publisher is **one OmniGraph node saved inside
`assets/usd/assembly_room.usd`** (binary USD crate; read with Isaac's bundled
`pxr`): `/Graph/Objects_TF/PublisherTF`, type
`isaacsim.ros2.bridge.ROS2PublishTransformTree`, topic `/tf`, ticked by
`OnPlaybackTick`, stamped from `ReadSimTime`.

Its configuration is the decisive part:

- `inputs:parentPrim -> ['/World']` — this only names the REFERENCE frame
  (frames are published relative to `World`); it does not select a subtree.
- `inputs:targetPrims -> [...]` — an **explicit list** of 22 prims: the two
  gripper contact frames, the ten FMB part prims (`/World/u_green`, ...,
  `/World/base1`, `/World/base2`), and the placement/hole/reorient frames
  (note the `base1/*`, `base2/*` children are each listed individually —
  even a child of a listed prim is not published unless itself listed).

**A newly created prim under `/World` therefore does NOT automatically get a
TF frame.** A prim gets TF only by being added to `targetPrims`.
`/World/WatchdogIntruders` and everything under it is outside every
TF-published subtree, so it is a safe intruder root — no relocation outside
`/World` is needed. (Operator confirmed this matches how the stage is set up:
a prim's pose is findable only if explicitly added to the Isaac-side
publisher.) The parts have no shared parent Xform; they sit directly under
`/World` next to `ur5e`, `simple_room`, `Camera`, etc.

`scene_tf_sync_node` publishes no TF: it is a TF *consumer*
(`Buffer`/`TransformListener`, `lookup_transform` at
`nodes/scene_tf_sync_node.py:470`) that mirrors registry objects into the
MoveIt planning scene. Other TF publishers, all robot/camera scoped:
`world→World` static identity and the camera static TF
(`sim_moveit_bringup.launch.py`), and `robot_state_publisher` for the URDF
links. Joint states come from `/Graph/ROS_JointStates` (no TF).

**VISION_DISCOVERY.md §2.1 / blocker #5 is stale on this point**: it claims a
disabled registry entry such as `red_cube` still gets TF from Isaac. In the
current stage `/World/red_cube` and `/World/green_cube` exist but are
*deactivated*, and neither is in `targetPrims` — re-activating the prim would
NOT produce a `red_cube` TF frame unless it were also added to the list. The
other half of that finding (registry `enabled` only gates collision geometry,
and `get_object_pose` ignores `enabled`) remains true and matters below.

### 0.2 Agent-reachable paths that could reveal a non-registry object

Audited every tool and MonitorContext channel. Summary: **no path enumerates
TF frames** — every listing comes from the registry or the cell config — but
**three by-name paths resolve raw TF** and would return a pose for any frame
name the agent passes, plus one registry quirk:

| Path | Source of ids | Leak? |
|---|---|---|
| `get_object_pose` (executor) | registry key lookup; unknown id → "not found in registry" | No for unknown ids, **but it ignores `enabled`**: `get_object_pose("red_cube")` resolves if that frame is ever published |
| `get_object_poses` (executor) | registry iteration, `enabled_only=True` forced | No |
| `get_scene_summary` (executor) | MoveIt planning scene; only writer is scene_tf_sync_node (registry-driven, skips disabled/frames) | No |
| `query_snapshot` (watchdog) | `recorder.object_ids()` / `frame_ids()` — registry + cell config only; unknown id → null | No |
| `get_contract` / `get_evidence` (watchdog) | task objects / recorder buffers; unknown ids → error or empty | No |
| `MonitorContext.object_ids/frame_ids/object_pose/frame_pose` and windows | recorder buffers (registry-derived) | No |
| **`MonitorContext.transform(parent, child)`** | **raw `tf2_ros.Buffer.lookup_transform` over live TF, no registry check** | **Yes, by name** (S2/S3, via first_call_result, dry-run return_value, fire value/detail, or the exception text) |
| **`arm_monitor` `relative_frame` primitive** | raw TF lookup of `parent_frame`/`child_frame`; arm result returns `reference_values.translation` | **Yes, by name** (S1/S2) |
| **Motion tools (`frame_id` args)** | MoveIt's own TF | **Yes, indirectly**: existence oracle via error text; pose via move-to-frame + `get_robot_state` |

TF error text also acts as a per-name existence oracle on every raw-TF path
(the tf2 message says whether a frame exists; it never lists frames).

Consequence for the injection design: an intruder is invisible to every
enumeration, and its *name* can only leak if the agent can guess it — or if it
collides with a registry id (`red_cube` resolves through `get_object_pose`
despite `enabled: false`). Hence the hard collision check in
`inject_intruder.py` against **all** registry object ids, `tf_frame` and
`lookup_frame` values (disabled entries included), and cell frames — and the
leak audit (§3) as the backstop that catches a name reaching any channel
anyway. Intruders are additionally created under `/World/WatchdogIntruders`,
which no TF publisher covers (§0.1), so there is no frame to look up even by
a guessed name.

## 1. record_observation

**Shared implementation:** `robot_backend/vision/observation.py`, mirroring
`vision/look.py`'s host seam (`ObservationDeps` dataclass + pure core).
Registered as a tool **on the WatchDog server only, in ALL scenarios**
(`watchdog_mcp_server.py`, plain `@mcp.tool()`). The S4 executor-side host is
a clearly marked hook in observation.py's module docstring ("S4 HOOK"): it
will construct `ObservationDeps(role="executor", ...)`; the payload's `role`
field, the per-role look registry, and the role-based self-emission rule are
already in place for it. Deliberately NOT added to the scene server.

```
record_observation(summary: str, category: str, location: dict | None,
                   first_seen_wall_ns: int | None, evidence_look_ids: list[str],
                   evidence_monitor_ids: list[str] | None = None,
                   identity: str | None = None, confidence: str = "medium",
                   requirement_impact: str = "none",
                   impact_detail: str | None = None,
                   observation_id: str | None = None,
                   status: str = "new") -> dict
```

- `category` ∈ {unlisted_object, object_moved, object_missing,
  other_unexpected}; `confidence` ∈ {low, medium, high}; `status` ∈ {new,
  update, resolved}; `requirement_impact` ∈ {none, violates, unknown} —
  "violates" requires `impact_detail` naming the requirement.
- `location`: `{"frame": "world", "x", "y", "z"}` and/or
  `{"pixel": {"u","v"}, "look_id"}`; at least one form required unless
  category is `object_missing`. The pixel form's `look_id` is validated like
  evidence ids.
- `observation_id`: server-generated on "new" (`obs-NNN-xxxxxxxx`); passing
  one on "new" is refused; required and existence-checked on update/resolved.
- `evidence_look_ids`: validated against the looks this role actually made in
  this process. `vision/look.py` registers every completed look's id into
  observation.py's per-role registry at the moment the look returns, so
  validation is in-process ground truth, not an event-topic scan. Unknown ids
  error, listing the valid ones. Empty is allowed; the result carries
  `unsupported_by_looks: true` plus a note.

**Event:** new `"observation"` type, `ObservationPayload` in
`events/schema.py` (all fields + `role` + `wall_ns` + `step_id`). Added to
`EventEnvelope`'s Literal, `make_event`, `EVENT_TYPE_PAYLOAD_CLASSES` (so
`KNOWN_EVENT_TYPES`, `available_event_types`, and the drift test pick it up
automatically), and `_WATCHDOG_EVENT_TYPES` with a look-style comment. In
`recorder.py` it follows "look" exactly: self-emission decided by
`payload.role` (the WatchDog is never woken by its own observation), and
wake-opt-in (`_WAKE_OPT_IN_EVENT_TYPES`) for a future executor-emitted one.
It is NOT a verdict: never calls `_note_finding`, never counts in
`findings_during_run`/`recovered` (asserted in tests).

The tool docstring is agent-facing and scene-generic: what an observation is,
that it is not a finding, that presence alone is not a violation, that a
requirement-breaking observation also needs a normal verdict, and the
update/resolve lifecycle.

## 2. Intruder injection — `scripts/isaac/inject_intruder.py`

Operator tool for Isaac's Script Editor. Pure-Python core (importable and
unit-tested outside Isaac) + an Isaac layer whose `omni.usd` / `pxr` /
`omni.kit.app` imports are lazy inside functions.

API: `inject(name, shape="cube"|"cylinder"|"sphere", size_m,
position_xyz_world, yaw_deg=0, color_rgb, physics, label, expected_impact:
"benign"|"violates", notes="", at_step=None, delay_s=0)`, `move(name,
position_xyz_world, yaw_deg=None, at_step=None, delay_s=0)`, `remove(name,
at_step=None, delay_s=0)`, `list_injections()`.

- Prims live under `/World/WatchdogIntruders` — confirmed OUTSIDE every
  TF-published subtree (§0.1), so no root change was needed.
- Names are validated NOW (at call time, whatever the trigger) against
  `load_forbidden_names()`: every registry object id, every `tf_frame` and
  `lookup_frame` (disabled entries like `red_cube` included), cell frames,
  and builtins (`world`, `World`, ...). Shape/size/color/impact likewise.
- `at_step`: resolves the live session dir from `~/.watchdog/current_session`
  and tails `<session>/events_scene.jsonl` by incremental file read (offset
  kept per pending action; `StepTriggerMatcher` buffers partial lines and
  skips corrupt ones), on an `omni.kit.app` update-stream subscription. Fires
  when a `begin_step` with the step_id appears. `delay_s` uses the same
  subscription with a monotonic deadline. Isaac's main thread is never
  blocked; the subscription is kept once created (releasing it from inside
  its own callback is not guaranteed safe in kit).
- **Ground truth:** one JSON line per EXECUTED action appended to
  `<session_dir>/ground_truth_injections.jsonl` via open-append + flush +
  fsync. Schema (every key always present, null when n/a):

```
{action, name, prim_path, shape, size_m, position_xyz_world, yaw_deg,
 color_rgb, physics, label, expected_impact, notes,
 trigger ("immediate" | "at_step:<id>" | "delay:<s>"),
 wall_ns (time.time_ns() at the moment the prim change was applied),
 wall_time_iso, session_dir, script_sha256}
```

- Never publishes to `/watchdog/events`, never touches ROS.
- Physics: `physics=True` applies `UsdPhysics.CollisionAPI` +
  `RigidBodyAPI`; `False` leaves a static visual prim.

Operator usage (full block at the top of the file):

```python
import sys; sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1/scripts/isaac")
import inject_intruder as ii
ii.inject("intruder_a", shape="cube", size_m=0.05,
          position_xyz_world=(0.4, -0.2, 0.76), color_rgb=(0.9, 0.1, 0.1),
          physics=True, label="red cube near base2", expected_impact="benign")
ii.move("intruder_a", (0.5, -0.1, 0.76), delay_s=10.0)
ii.remove("intruder_a", at_step="place_hex_red")
ii.list_injections()
```

## 3. Leak audit — `scripts/audit_intruder_leak.py <session_dir>`

Reads `ground_truth_injections.jsonl`; search terms are each intruder's name
and prim leaf name. Scans:

- **(a)** the run's rosbag `/tf` and `/tf_static` child/parent frame ids, via
  `rosbag2_py` + `tf2_msgs` when importable (every `metadata.yaml` directory
  under the session dir); otherwise `checked` says so and b–d stand alone;
- **(b)** `mcp_calls_watchdog.jsonl` / `mcp_calls_executor.jsonl` (whole call
  records, arguments and results);
- **(c)** every `events_*.jsonl`;
- **(d)** `authored_monitors/*.py` (any mention at all is a hit).

Exemptions ("outside look/observation text"): event lines whose `event_type`
is `look` or `observation`, and call records whose `tool` is `look` or
`record_observation` — the legitimate camera-mediated channel. Everything
else naming an intruder is a leak, because a prim name is never visible in
pixels.

Output `{"leak": bool, "hits": [...], "checked": [...]}` (+ intruder_names,
record count, session_dir) to stdout and
`<session_dir>/intruder_leak_audit.json`. Exit code: 0 clean, 1 leak, 2
usage/missing ground truth. A run with `leak: true` is excluded from
observation scoring.

## 4. Small items from Stage 2 live testing

- **4a.** `purpose="observation"` now requires `threshold_param` naming one of
  `params` — enforced in `_prepare_authored_monitor` on the **arm path only**
  (`for_arm=True`; the dry run passes `for_arm=False` because it records no
  armed event and deliberately never takes threshold_param). The error shows
  the params form. Clause behaviour unchanged. Live motivation: the recorded
  threshold was 0.0 while the agent's real 0.02 sat only in params.
- **4b.** `arm_custom_monitor` and `dry_run_custom_monitor` docstrings now
  carry the explicit params-form line: `e.g. params={"change_m": {"value":
  0.02}} -- every entry is a mapping with a "value" key, never a bare number`
  (live: the first dry run passed a bare number).

## 5. Files changed / added

| File | Change |
|---|---|
| `robot_backend/events/schema.py` | ObservationPayload; "observation" in envelope Literal, make_event, payload-class map, watchdog origin set |
| `robot_backend/watchdog/recorder.py` | observation in wake-opt-in set; role-based self-emission for look AND observation |
| `robot_backend/vision/observation.py` | **new** — shared core, ObservationStore, per-role look registry, S4 hook |
| `robot_backend/vision/look.py` | registers each completed look's id in the observation look registry |
| `robot_backend/mcp_servers/watchdog_mcp_server.py` | record_observation tool (all scenarios); `_OBSERVATIONS` store; observation threshold_param arm check (`for_arm`); 4b docstring lines |
| `scripts/isaac/inject_intruder.py` | **new** — operator injection tool |
| `scripts/audit_intruder_leak.py` | **new** — leak audit |
| `test/test_observation.py`, `test/test_inject_intruder.py`, `test/test_audit_intruder_leak.py` | **new** — 42 tests |

## 6. Test results

```
test_observation + test_inject_intruder + test_audit_intruder_leak
  -> 42 passed

Stage 2 set + STAGE1_VERIFY §2 targeted list + test_skill_event_coverage
(test_run_ended_terminal_event, test_prompt_variants,
 test_session_and_image_buffer, test_scenario_gating,
 test_threshold_provenance, test_authored_monitor, test_authored_lane,
 test_authored_parity, test_recorder_windows,
 test_authored_purpose_and_rate, test_array_lane, test_authored_validator,
 test_skill_event_coverage, test_event_bus, test_engine, test_monitors,
 test_watchdog_run_verdict, test_final_run_verdict_cadence,
 test_watchdog_forced_termination)
  -> 326 passed
```

Note: STAGE1_VERIFY §2's one expected failure
(`test_s4_task_matches_derivation[fmb_assembly_2_s4.executor.yaml]`) no
longer fails — the S4 derivation was evidently regenerated since.

`test_flake8` / `test_pep257` fail before and after this change (verified by
stash): the known pre-existing lint pair (HANDOFF §6). The new files add no
flake8 errors; the pep257 complaints on them (D213 etc.) are the house
docstring style, endemic repo-wide (1708 occurrences).

## 7. Unresolved / deferred

- The three by-name TF leak paths (§0.2: `MonitorContext.transform`,
  `relative_frame` arming, motion `frame_id`) are left open by design this
  stage — the audit detects use rather than the servers preventing it. A
  registry allowlist on `recorder.lookup_transform` (and an `enabled` check in
  `get_object_pose`) would close them if scoring ever shows probing.
- VISION_DISCOVERY.md §2.1/blocker #5's "disabled entries still get TF" claim
  should be corrected to "disabled entries get TF only if left in
  targetPrims; currently they are not" (not edited this stage).
- The Isaac layer of `inject_intruder.py` (prim creation, xform ops, the kit
  update subscription) is verified by reading only; first live use should
  start with one immediate `inject` + `list_injections()` before trusting
  `at_step` triggers.
- `ground_truth_injections.jsonl` says nothing about whether the camera could
  SEE the intruder (occlusion); scoring that needs the frames.
- Scorer integration (excluding `leak: true` runs, matching observations to
  ground truth) is Stage-3-scorer work, not done here.
