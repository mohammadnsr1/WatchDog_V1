# Stage 1 + fix pack — offline verification (2026-09-30)

Verification only; no source file was edited. Everything ran with
`ros2_ws/.venv/bin/python3` under `unset VIRTUAL_ENV`, ROS Jazzy + install
space sourced, `_ROBOT_BACKEND_MCP_VENV_READY=1`, and
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` for pytest.

**Environment note:** the install space on this machine predates the Stage 1
pull (no `robot_backend/vision/`), and `colcon` was out of scope, so every
command ran with `PYTHONPATH="$PWD/src/robot_backend:$PYTHONPATH"` so the
pulled sources are what was verified. Before a live run, `colcon build
--packages-select robot_backend` is still required.

## 1. Import + tool registration

`robot_backend.vision.look`, `watchdog_mcp_server` and `scene_mcp_server` all
import cleanly. Registered tools via `asyncio.run(mcp.list_tools())`, one
interpreter per scenario:

| Surface | Result |
|---|---|
| S1 | `look`, `arm_monitor`, `available_monitor_definitions` present; no `arm_custom_monitor`, no `dry_run_custom_monitor` — **as expected** (19 tools) |
| S2 | `look`, `arm_monitor`, `arm_custom_monitor`, `dry_run_custom_monitor` all present — **as expected** (21 tools) |
| S3 | `look`, `arm_custom_monitor`, `dry_run_custom_monitor` present; no `arm_monitor`, no `available_monitor_definitions` — **as expected** (19 tools) |
| scene | `look` present — **as expected** (14 tools: begin_step, end_run, end_step, get_object_pose, get_object_poses, get_object_spec, get_scene_summary, look, mark_phase, recover_scene, reset_object_state, reset_tf, set_permitted_contacts, sync_scene) |

No deviations.

## 2. Targeted tests

```
test_run_ended_terminal_event, test_prompt_variants, test_session_and_image_buffer,
test_scenario_gating, test_threshold_provenance, test_authored_monitor,
test_authored_lane, test_authored_parity, test_recorder_windows
-> 1 failed, 146 passed in 2.05s
```

The one failure is exactly the expected one:
`test_prompt_variants.py::test_s4_task_matches_derivation[fmb_assembly_2_s4.executor.yaml]`.

Exact difference (steps compared field by field between
`fmb_assembly_2_s4.executor.yaml` and `fmb_assembly_2.executor.yaml`; 12 steps
in both, one step differs, one field):

```
step[2] place_u_brown, field 'target_offset':
  S4 file : absent (null)
  source  : {"position": {"x": 0.02, "y": 0.0, "z": 0.0},
             "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
```

Cause: commit 564da5c ("Enhance fmb_assembly_2 executor with target offset
configuration") un-commented `target_offset` on `place_u_brown` in the source
executor task but the derived S4 copy was not regenerated. Not fixed, per the
task instruction.

## 3. Full suite

`pytest -q --continue-on-collection-errors src/robot_backend/test`
(plain `pytest -q` aborts at the known `test_grasp_skill` collection error):

```
14 failed, 435 passed, 1 skipped, 1 error (collection) in 13.26s
```

Mapping against HANDOFF.md §6's known set (6 event-coverage, 6
contract/registry, 1 grasp_skill collection, 1 rg2 round-trip, 2 lint):

**In the known set:**
- `test_grasp_skill.py` collection error (`AIR_CLOSE_FLOOR_EFFORT` gone) — known.
- `test_rg2_geometry.py::test_width_lookup_round_trip_matches_requested_width`
  (`assert 7.9e-05 == 0.0 ± 1e-06`) — known.
- `test_flake8`, `test_pep257` — the 2 known lint failures.
- 5× `test_pick_contract.py::TestLoadFromRegistry` (stale cube values, e.g.
  `assert 130.0 < 95.0`, `assert 70.0 == 0.18 ± 1.8e-07`) — the known
  contract/registry family.
- 4× `test_registry_loader.py` — 3× `FileNotFoundError:
  model_registry_mesh_example.yaml` (file removed 2026-09-14 in 52a9a1f,
  pre-Stage-1) and 1× `KeyError: 'spec'` — contract/registry family.
  (pick_contract + registry_loader total 9 vs §6's count of 6; under the venv
  FastMCP imports, so the split between §6's "event-coverage" and
  "contract/registry" buckets differs from the system-python run §6 was
  counted under. None of these touch Stage 1 code.)
- The 6 "event-coverage (FastMCP not importable in system python)" failures
  do NOT reproduce under the venv (FastMCP imports there); 5 of them pass.

**NOT in the known set (2):**

1. `test_skill_event_coverage.py::TestSceneServerCoverage::test_all_tools_covered`
   ```
   AssertionError: scene MCP tool(s) with no event coverage and no allowlist
   entry: ['get_object_spec', 'look']
   ```
   Under system python this test was one of §6's FastMCP-import failures, so
   its real assertion was invisible until now. `look` is a Stage 1 regression
   surface: the scene server's new `look` tool is neither event-decorated in
   the test's terms nor on its allowlist (it emits its own `look` event, so
   the fix is likely an allowlist/coverage-map entry, not new events).
   `get_object_spec` predates Stage 1.

2. `test_prompt_variants.py::test_s4_task_matches_derivation[fmb_assembly_2_s4.executor.yaml]`
   — the expected failure detailed in §2 (`steps differs` on
   `place_u_brown.target_offset`).

## 4. Offline unit checks

a. `depth_to_grayscale_png` on a synthetic 480×640 float32 depth array with a
   NaN block: returns PNG bytes starting `b'\x89PNG\r\n\x1a\n'` and a
   2-element clip range (`[0.432, 1.968]`, the p2/p98 of the valid pixels).
   **PASS.**

b. `downscale` on 1080×1920×3 with `max_width=640`: returns shape
   `(360, 640, 3)` — width ≤ 640 and the 16:9 aspect is preserved exactly
   (integer stride 3). **PASS.**

c. Recorder role-aware `look` filtering, on an `EvidenceRecorder(node=None,
   registry={"objects": {}})` exactly as `test_recorder_windows` builds one,
   feeding real `make_event("look", LookPayload(role=...))` envelopes through
   `_on_event_msg`:
   - executor-role look via `pop_wait_events(None)`: **not delivered** (and
     retained in the queue, not dropped);
   - executor-role look via `pop_wait_events({"look"})`: **delivered**
     (including the earlier retained one — `['look', 'look']`);
   - watchdog-role look: **never queued for waiting** (a subsequent opt-in
     pop returns `[]`).
   **PASS** on all three behaviors. (Import note: the package exports a
   `look()` function, so the module must be imported as
   `importlib.import_module("robot_backend.vision.look")` — `from
   robot_backend.vision import look` yields the function.)

## 5. Settings allowlists

Both settings files allow at the **server** level, not per tool:

- `watchdog.settings.json` → `"allow": ["mcp__robot-backend-watchdog"]`
- `executor.settings.json` → `"allow": ["mcp__robot-backend-motion",
  "mcp__robot-backend-gripper", "mcp__robot-backend-scene"]`

A server-level entry covers every tool that server registers, so in the same
naming form as the existing entries **nothing is missing**: the watchdog's
`look` and `dry_run_custom_monitor` are covered by
`mcp__robot-backend-watchdog`, and the scene server's `look` by
`mcp__robot-backend-scene`. (Per-tool equivalents, were the files ever
switched to that form: `mcp__robot-backend-watchdog__look`,
`mcp__robot-backend-watchdog__dry_run_custom_monitor`,
`mcp__robot-backend-scene__look`.) Nothing was added.

## Camera receive fix (2026-09-30, follow-up to the live S2 failure)

### Root cause

`CameraStreams.attach()` and `ImageBufferManager.attach()` gated EVERY
subscription on a `get_topic_names_and_types()` snapshot taken milliseconds
after node creation (`robot_backend/watchdog/image_buffer.py`, pre-fix lines
460-501 / 281-293). That snapshot races DDS discovery: a topic not yet
discovered at attach was recorded "missing" and never subscribed, so look()'s
slots stayed empty for the whole run while the camera published all along.

Reproduced offline (domain 77): building the vision path via the server's own
construction code with the publishers appearing after attach yields
`camera streams: subscribed to []` and the exact live failure strings --
rgb "no RGB frame available" (age null), depth "no depth image has arrived on
/camera/depth/image_raw", "camera_info is absent".

A second live-blocking bug surfaced by the new end-to-end test:
`camera_info_to_intrinsics` evaluated `msg.k or msg.K`; rosidl exposes
`CameraInfo.k` as a numpy array, whose truth value raises, and the bare
`except` turned that into `return None` -- intrinsics would have parsed from
NO real CameraInfo message even once subscribed.

### Changes (src line numbers post-fix)

`robot_backend/watchdog/image_buffer.py`
- `CameraStreams.attach` (485): subscribes unconditionally to every configured
  topic; discovery is diagnostic only (log + `missing_topics`). Subscribes
  BOTH camera_info candidates (`_info_candidates`, 598); `_on_info` (633)
  records which topic filled the slot and prefers the namespaced one.
- Per-stream instrumentation: `_count_received` (448) / `_count_error` (456)
  replace every silent swallow in `offer_rgb`, `_on_rgb_msg`, `_store`
  (first error logged, then every 100th); `stats()` (813) reports per stream:
  subscribed topic(s), QoS reliability, received count, last receive wall
  time, error count and first error; `never_received_text()` (853) renders
  "no message received on <topic> since attach at <iso> (<n> s ago);
  received=<n>, errors=<n>".
- `ImageBufferManager.attach` (265): same unconditional-subscribe change for
  the RGB ring-buffer path; compressed variants recognised by name.
- `camera_info_to_intrinsics` (962): no numpy-truthiness (`k or K`) -- explicit
  None checks for `k`/`K` and `d`/`D`.

`robot_backend/vision/look.py`
- `LookResult.streams` (134) + `"streams"` block in `text_payload`, filled
  from `CameraStreams.stats()` in `perform_look` (324), so a live failure
  explains itself in the look's own text part.
- rgb never-received failures use `never_received_text` (358).

`robot_backend/mcp_servers/watchdog_mcp_server.py`
- `_build_camera_pipeline(node, image_cb_group)` (601): the whole camera path
  factored out of `_ensure_watchdog` -- one CameraStreams per process, created
  before the manager, reached only through the returned context -- and the
  exact construction the end-to-end test builds.
- `get_execution_state` returns `now_wall_ns` (1457), and `capture_frames`'
  docstring names it as the way to build a "last N seconds" window (item 4).

Also: scene-server `look` added to `SCENE_ALLOWLIST` in
`test_skill_event_coverage.py` ("emits its own look event"), and
`agents/tasks/fmb_assembly_2_s4.executor.yaml`'s `place_u_brown` gained the
source file's `target_offset` so the S4 derivation test passes again.

### Proof: fail before, pass after

Pre-fix repro (scene path built via `_ensure_vision`, publishers created
after attach, domain 77):

```
camera streams: subscribed to []; missing ['/camera/depth/image_raw',
  '/camera/depth/camera_info', '/camera/depth/points']
views_ok    : []
views_failed: rgb 'no RGB frame available'; depth 'no depth image has
  arrived on /camera/depth/image_raw ... camera_info is absent ...'
ages: {'rgb': None, 'depth': None, 'camera_info': None, 'pointcloud': None}
```

Post-fix, the committed end-to-end tests (`test_camera_streams_live.py`,
same publish-after-attach ordering) pass: both views ok,
`depth_source == "depth_image"`, camera K matches (fx 554.26), stream
counters show received > 0 and errors == 0, on BOTH the watchdog path
(`_build_camera_pipeline`) and the executor path (`_ensure_vision`). The
intermediate run where rgb/depth passed but `camera` was None is what exposed
the `CameraInfo.k` truthiness bug; it passes after that fix.

### Test results

- `test_camera_streams_live.py`: 5 passed (3a watchdog end-to-end, 3b scene
  end-to-end, 3c unconditional-attach units x2, never-received text).
- STAGE1_VERIFY §2 targeted list + `test_skill_event_coverage` +
  `test_camera_streams_live`: **160 passed, 1 failed** -- the one failure is
  `test_skill_event_coverage::test_all_tools_covered` on `get_object_spec`
  only, the PRE-EXISTING gap this task says to leave alone.
  `test_s4_task_matches_derivation[fmb_assembly_2_s4.executor.yaml]` passes
  again after the target_offset sync.

NOTE: `colcon build --packages-select robot_backend` is required before the
next live run; these fixes exist in src only until then.
