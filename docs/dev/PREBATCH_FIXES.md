# Pre-batch fixes from the 2026-10-01 smoke runs

Sessions referenced: S1-low `20261001T215105Z`, S2-low `20261001T230622Z`,
S3-low `20261002T002816Z`, S2-medium (Opus) `20261001T213529Z`. All four are
present. Everything below ran with `ros2_ws/.venv/bin/python3` (unset
VIRTUAL_ENV, ROS Jazzy + install sourced, `_ROBOT_BACKEND_MCP_VENV_READY=1`,
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`); no colcon, no ros2 CLI, no launch, no
Isaac, no servers.

## 1. TF time alignment — VERDICT: ARTIFACT

### 1a. What the recorder actually samples

`recorder.py::_on_tf_timer` (20 Hz) loops over every object chain and cell
frame and calls `lookup_transform(lookup_frame, tf_frame, rclpy.time.Time())`
— **Time(0), i.e. each chain's own latest available TF time, resolved
independently per lookup**. The stamp stored on every sample is
`tick_wall_ns = wall_ns()` (the recorder's wall clock at tick start, shared
across the tick); **the TF time actually used (`transform.header.stamp`) is
discarded** (`_transform_to_pose_tuple` keeps only the wall stamp). So two
samples from one tick share a wall stamp but are evaluated at different TF
times, and nothing in the record can show by how much.

A single `lookup_transform(tcp, obj, Time(0))` — what the library
`relative_frame` monitor and `ctx.transform` do — is internally
time-consistent: tf2 evaluates the whole tcp→world→obj chain at its latest
*common* time. The artifact only exists when a relative quantity is assembled
from two independently-sampled buffer entries, which is exactly what the S3
authored monitors did: `dist(ctx.frame_pose("tcp"), ctx.object_pose(obj))`
(see `authored_monitors/mon_cfba56a5db8f.py` in the S3 session).

### 1b. Bag reconstruction (S3 bag, /tf + /tf_static, both fire windows ±3 s)

Method: stream the bag in receive order into a tf2 buffer; at every 20 Hz
tick compute (i) the recorder's way — `world→tcp` at Time(0) and `world→obj`
at Time(0) as two separate lookups, distance between the two translations —
and (ii) the time-consistent way — one `tcp→obj` lookup at Time(0), norm of
its translation. Per-chain times are read off each lookup's own
`header.stamp`.

| Window (fire ±3 s, 120 ticks) | recorder-style apparent drift | time-consistent drift | tcp-vs-object stamp offset | chain rates |
|---|---|---|---|---|
| `u_brown` place_approach, fire 00:32:13.76 (value 0.0209 m) | **17.7 mm** (dist 0.0891–0.1068 m) | **0.1 mm** (0.0911–0.0912 m) | −50.0…+16.7 ms, mean −23.9 ms | u_brown 33.7 Hz, robot links (wrist_3) 11.2 Hz |
| `line_green` lift, fire 00:36:40.66 (value 0.0204 m) | **26.1 mm** (0.0564–0.0825 m) | **1.1 mm** (0.0749–0.0760 m) | −50.0…+16.7 ms, mean −19.2 ms | line_green 33.8 Hz, robot links 11.3 Hz |

Series excerpts around each fire (mm above each series' own minimum):

```
u_brown    -0.33s rec= 7.1 con=0.1 | -0.13s rec= 9.6 con=0.0 | +0.07s rec=17.7 con=0.0 | +0.27s rec=5.3
line_green -0.44s rec=14.3 con=1.1 | -0.04s rec= 6.0 con=1.1 | +0.16s rec=25.5 con=1.1 | +0.36s rec=14.7
```

The two monitors' trigger values (20.9 mm, 20.4 mm against a 20 mm bound)
sit inside the recorder-style envelopes (17.7 mm, 26.1 mm) while the real
relative motion never exceeded 1.1 mm. Note the laggard is the **robot**
chain: `robot_state_publisher` republishes at ~11 Hz against the Isaac object
chains' ~34 Hz, so during arm motion the object chains run up to ~50 ms ahead
of the tcp sample — at commanded tool speeds that is tens of millimetres of
phantom relative displacement, largest exactly when the arm moves fastest
(approach and lift), which is when both fires happened.

### 1c. Verdict

**ARTIFACT**, on the stated criterion, in both windows: recorder-style drift
17.7 / 26.1 mm (> 10 mm) against time-consistent 0.1 / 1.1 mm (< 5 mm).
Consistent with the S1 contrast (the library `relative_frame` monitor on the
same 0.0760 m tcp→line_green reference never fired through the same lift —
its single-lookup path is time-consistent), and it does not touch the
S2-medium jam fires, where a library and an authored monitor using
time-consistent lookups agreed within 32 ms about a real event.

Under aligned sampling the two S3 fires would not have occurred: the aligned
tcp↔object distances move 0.1 mm (u_brown) and 1.1 mm (line_green) across
their windows, both far under the 0.02 m bound the monitors carried.

## 2. TF time alignment — FIX (recorder common-time sampling)

`recorder.py::_on_tf_timer` now samples in two passes: pass 1 resolves every
chain's latest transform and reads the TF time off each lookup's own header;
the tick's common time is **the latest time available to every healthy chain**
(the minimum of the per-chain latests); pass 2 re-samples every chain at that
one time. Guards: a chain lagging the freshest by more than
`TF_COMMON_TIME_MAX_LAG_SEC` (1.0 s) is excluded from the common-time
computation and sampled at its own latest (a dead chain must not drag every
sample into the past — `tf_staleness_counters` is its reporting channel, and
it still works exactly as before); a pass-2 re-lookup that fails falls back to
the chain's pass-1 latest, honestly stamped. The common time lags "now" by at
most the slowest healthy chain's publish interval (~90 ms at the robot chain's
11 Hz) — the same order of staleness a Time(0) sample of that chain already
had, so detection/halt latency is unchanged. Library monitors
(`stationarity`, `region` via the dict samples; `relative_frame` via its own
single time-consistent lookup) behave as before.

**tf_stamp_ns** is stored on every sample (appended as the pose tuple's 10th
element so no consumer's unpacking changes; `_pose_tuple_to_dict` accepts both
shapes) and flows through `latest_object_pose`/`latest_frame_pose`, the
windows, `get_evidence`, `query_snapshot`, and `ctx.object_pose`/
`ctx.frame_pose` and their windows (their `_copy_pose_sample` passes keys
through). The docstring sentence "Samples from one recorder tick share one TF
time (tf_stamp_ns)." is on `query_snapshot`, `get_evidence` and
`arm_custom_monitor`'s ctx entries (guard-tested).

Test: `test_tf_time_alignment.py` (5 tests) — a synthetic tf2 buffer where the
tcp chain leads a rigidly-attached object chain by 150 ms at 0.2 m/s: the old
latest-per-chain method reads the rigid 0.100 m separation as 0.130 m (the
lag-induced 30 mm), aligned sampling reads 0.100 m with < 1e-6 drift across
ticks; the stale-chain guard and both tuple shapes are covered.

**How the two S3 fires would have evaluated under aligned sampling: no fire.**
The aligned tcp↔object distances move 0.1 mm (u_brown) and 1.1 mm
(line_green) across their ±3 s windows, far under the monitors' 0.02 m bound
(§1b's time-consistent columns are exactly the aligned-sampling values).

## 3. Run note removal

- `agents/tasks/fmb_assembly.watchdog.yaml` / `fmb_assembly_2.watchdog.yaml`:
  the `run_notes` block naming `relative_frame` + `arm_custom_monitor` (the
  text all three scenarios rendered) replaced with `run_notes: ""` plus a
  comment saying why it stays empty. No shared file names a scenario-gated
  tool any more (grep-verified: the only remaining hits were these two).
- `agents/agent_harness/task_render.py`: an empty `run_notes` now drops the
  whole "## Run notes" section from the rendered task instead of leaving a
  dangling header (one renderer, both templates).
- S4 tasks regenerated (no instruction_spec/object_ids change resulted).
- Guard tests (`test_rendered_surface.py`, rendering through the real
  `role_context.build_rendered_task` for every scenario and both assemblies):
  S1 renders name neither `arm_custom_monitor` nor `dry_run_custom_monitor`;
  S3 renders name neither bare `arm_monitor(` nor
  `available_monitor_definitions`; no rendered task (watchdog or executor,
  any scenario) contains a "Run notes" section or any monitor-arming tool
  name.

## 4. Agent model settings on record

Before this change: both launchers read `model`/`effortLevel` from their
`--settings` JSON (`watchdog.settings.json` / `executor.settings.json`,
currently claude-sonnet-5 / low for both) and passed no model flags; the
settings file was the only selector.

Now: `run_watchdog.sh` and `run_executor.sh` take `MODEL` and `EFFORT` env
vars, defaulting to the settings-file values, and pass them as explicit
`--model` / `--effort` flags — both verified against the installed CLI
(2.1.287: `--model <alias|full name>`, `--effort <low|medium|high|xhigh|max>`),
and flags are the highest-precedence form, so the launch line is authoritative.
Each launcher writes `<session_dir>/agent_config_<role>.json` with
`{role, scenario, assembly, model, effort, claude_cli_version,
launched_wall_time_iso}` (executor `scenario` is null — S1–S3 live on the
watchdog side; a dangling session pointer skips the write with a warning).
Verified end-to-end with a stubbed `claude`, including the `EFFORT=high`
override. RUN.md documents it and states the batch rule: the executor's model
and effort are identical for every run of a batch; only the WatchDog's effort
varies.

## 5. Scope of registry-bound tools

The sentence "Covers only the objects and frames registered for this cell; it
cannot show anything else that is in the workspace." added verbatim
(grammar-only adaptation) to:

- WatchDog tool docstrings and their system-prompt entries: `query_snapshot`,
  `get_execution_state`, `get_evidence`, `get_contract` (4 entries per
  rendered watchdog prompt, every scenario — guard-tested by normalized
  count).
- `arm_custom_monitor`'s ctx block: one NOTE covering `object_ids`,
  `frame_ids`, `object_pose`, `frame_pose` and their windows.
- Executor tool docstrings: `get_object_pose`, `get_object_poses`,
  `get_scene_summary` (no per-tool prompt entries exist for these, so
  docstrings only).

Nothing about look, the camera, or detection was added. Variants regenerated;
guard tests in `test_rendered_surface.py`.

## 6. Held-span semantics

Confirmed from `engine.on_skill_event` before writing: held becomes the
object id at `grasp_skill` skill_start; `ok: true` at skill_end confirms it; a
failed grasp clears it; `release_skill` skill_start clears it and
`_release_disarm_slip_monitors` disarms **only** `monitor_type ==
"relative_frame"` with that subject (reason `auto_disarm_release`) — authored
monitors are untouched, exactly matching the S2-medium evidence (library slip
monitor auto-disarmed at 21:47:51.256; the authored one fired at 21:47:51.431
as the jaws opened). Both statements now in `arm_custom_monitor`'s ctx block
(`ctx.held_object()` entry), guard-tested. No suggested code anywhere in the
added text.

## 7. Arming vs inspecting

Added to `arm_custom_monitor`'s docstring, directly under the summary line:
"Arming is monitoring and is recorded as such. To inspect what ctx returns
without arming, use dry_run_custom_monitor." The ctx.transform entry now
states the exact shape `{"translation": {"x","y","z"}, "rotation":
{"x","y","z","w"}}`, the frame convention (the child frame's pose expressed in
the parent frame), and that it is a single time-consistent lookup. **One
deviation from the item text**: the item said to document "... or None", but
`ctx.transform` never returns None — a lookup that cannot be resolved raises
(authored.py's `transform`) — so the docstring states that instead of a
None contract that does not exist.

## 8. Library threshold provenance parity

`arm_monitor` takes an optional `object_id`, used ONLY for threshold
provenance when the primitive cannot resolve an object itself (the pre-grasp
`effort_event` case from the smoke runs, where the agent's `object_id` inside
params was silently ignored). Resolution order (`_resolve_provenance_object`):
the primitive's own subject ("params"), else the held part ("held"), else the
declared id ("declared"). The chosen source is recorded as
`provenance_object_source` on `monitor_armed` and `monitor_fire` (new
defaulted schema fields; engine restates it on fires via `_PROVENANCE_FIELDS`
and returns it in the arm result). An unregistered declared id is rejected
naming the registry's objects; a declared id conflicting with the primitive's
own resolution loses, and the arm result carries `object_id_conflict`
{declared, used, note}. Monitor behaviour, fire attribution and held-object
logic unchanged. Docstring: one line saying what object_id is for.

Tests (`test_threshold_provenance.py`, +6): pre-grasp effort_event with
object_id verifies against that object's `limits.max_clamp_effort`
(threshold_verified true, source "declared"); without object_id the result is
byte-identical to today ("object_unresolved"); unknown id rejected with
nothing armed; declared loses to held and to a params subject with the
conflict reported; the fire payload carries the source.

## 9. Intruder visibility in ground truth

`inject_intruder.py` pure core gains `project_world_point` /
`camera_view_fields`, built on documented constants: extrinsics = the static
world→camera_depth_optical_frame from `cameras.yaml` (t=(0.25,−0.2,1.7),
q_xyzw=(0,0.99863,−0.05234,0)), intrinsics 640×360, fx=fy=554.2563, cx=320,
cy=180, verified against `/camera/depth/camera_info` on 2026-09-30. Every
inject and move ground-truth record now carries `pixel_uv`, `in_camera_view`
(centre inside the image with a 20 px margin) and `edge_margin_px`; remove
records keep the keys as null. `inject()` warns and proceeds when
`in_camera_view` is false; `list_injections()` shows the fields (it prints
the records). Projection matches the reference points to < 0.5 px:
world (−0.5, 0.1, 0.035) → (575.9, 341.2) [ref ~(576, 341)];
(0.25, −0.38, 0.0) → (320.0, 179.6) [ref ~(320, 180)]. The S2/S3 smoke
placements around x≈−0.5 indeed land with edge_margin < 20 px — which is why
those intruders sat on the image's bottom-right edge. Tests added
(reference pixels within 5 px, margins, record fields).

## 10. Failed record_observation calls — diagnosis: errors were clear, no fix

S2-medium, `mcp_calls_watchdog.jsonl`, all three calls carrying the same
base1-displacement observation:

| Time (UTC) | Offending arguments | Error returned |
|---|---|---|
| 21:44:28 | `requirement_impact: "unclear"` AND `observation_id: "obs_base1_shift"` with `status: "new"` | `requirement_impact must be one of ['none', 'violates', 'unknown'] (got 'unclear').` |
| 21:44:37 | `observation_id: "obs_base1_shift"` still passed | `observation_id is server-generated on status='new'; do not pass one. To change an existing observation, use status='update' or 'resolved' with its id.` |
| 21:44:45 | both fixed | ok — the observation event published |

Each error named exactly what to change and the agent corrected it on the
immediately following call, so per the item's own criterion no message was
changed and no validation rule is wrong. The one cost: the first call carried
TWO invalid arguments and validation surfaces them serially, so fixing the
first revealed the second — one extra round trip (~8 s). Left as-is
deliberately: collecting all violations per call would restructure
validation for a marginal gain; noted here so a future pass can weigh it.

## 11. Test results

`regenerate_variants.py`: all derived files up to date after the edits — with
one real fix: the script now also copies the `steps:` block verbatim from the
source executor task into the S4 files (with a parse round-trip check). That
closes the pre-existing `test_s4_task_matches_derivation[fmb_assembly_2_s4]`
failure (commit c390b1d commented out `target_offset` on `place_u_brown` in
the source; the derived copy had kept it), through the sanctioned
never-hand-edit path. `fmb_assembly_2_s4.executor.yaml` was rewritten by the
script accordingly.

```
Item-11 matrix (prompt_variants incl. new guards, scenario_gating,
rendered_surface, observation, inject_intruder, audit_intruder_leak,
recorder_windows, tf_time_alignment, engine, monitors,
threshold_provenance, authored_* x5, array_lane, run_ended_terminal_event,
session_and_image_buffer, skill_event_coverage, event_bus,
watchdog_run_verdict, final_run_verdict_cadence,
watchdog_forced_termination)                      -> 407 passed, 0 failed

Full suite (--continue-on-collection-errors)      -> 12 failed, 562 passed,
                                                     1 skipped, 1 error
```

The 12 + 1 are exactly the pre-existing known set (2 lint, 5 pick_contract,
4 registry_loader, 1 rg2 round-trip, 1 grasp_skill collection error) —
unchanged from the pre-change baseline, which was 527 passed; every added
test passes and nothing regressed.

## Unresolved

- The S3/S1-low/S2-low runs that motivated items 1-3 predate these fixes;
  their recorded monitor fires remain artifacts on the record (the report
  above is the correction).
- The scorer still needs the Stage 3/4 integration (authored coverage,
  observation-purpose fires, ground truth + leak audit) — unchanged by this
  task, tracked in PRE_SMOKE_VERIFY.md §4.
- record_observation validation surfaces one error per call (see item 10) —
  acceptable, revisit only if it costs real turns in batches.
- The aligned sampling takes ~2x TF lookups per tick (60 at 20 Hz) —
  negligible against tf2's C++ buffer, not measured live yet; the first live
  run should glance at tf_staleness_counters staying flat as usual.

