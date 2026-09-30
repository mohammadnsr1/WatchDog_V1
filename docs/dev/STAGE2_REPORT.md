# Stage 2 — the camera array lane for authored monitors (2026-09-30)

Scene observation Stage 2: authored monitors can now read camera arrays (rgb,
depth, intrinsics, deprojection) on their own rate-scheduled lane, and a
monitor can be armed as an "observation" trigger whose fire is a cue to look,
never a finding. Built inside the existing authored framework (validator,
budgets, strikes, MonitorContext, `_prepare_authored_monitor`), no parallel
framework — the future S4 executor-side host reuses all of it: everything new
lives in `watchdog/authored.py`, `watchdog/engine.py` and
`watchdog/image_buffer.py`, none of which import mcp_servers.

## 0. Read-first summary (state of the framework this builds on)

- `authored.py`: `validate_source` (AST subset: one `check(ctx)`, literal
  top-level assigns, no import/while/class/underscore names/forbidden
  builtins), `compile_check` (whitelisted `__builtins__` + `math`),
  `run_with_budget` / `run_with_budget_counted` (settrace line budget +
  wall time, `usage` out-param for the dry run), `json_safe`/`_json_safe`,
  `MonitorContext` (recorder/engine views, `{value, source}` params,
  persistent `state`, `channels_read`), `AuthoredMonitor` (validate → compile
  → arm-time first call = reference capture; `evaluate()` applies the
  None-or-`{"value","detail"}` contract; `dry_call()` returns the raw value).
- `engine.py`: scalar authored lane = one daemon thread woken by
  `on_tf_tick`/`on_joint_state`, 20 ms wall budget, 5-strike
  `authored_timeout`, `authored_bad_return`/`authored_error` disarms with
  `detail`; fires go through the same `_handle_fire` as library monitors.
- Server: `_prepare_authored_monitor` (shared build+provenance path,
  `(monitor, err)` return), `arm_custom_monitor` (arm + save source),
  `dry_run_custom_monitor` (multi-tick, 50 ms spacing, usage per tick).
- `CameraStreams`: latest-only rgb/depth/camera_info/pointcloud slots,
  unconditional attach (Stage 1 fix), per-stream stats, 1.0 s staleness.

## 1. Camera accessors (authored.py)

`MonitorContext` (ctor now takes `camera`, `rate_hz`; lines 491-604):
- `rgb()` (510) → HxWx3 uint8, `writeable=False`, or None (absent/stale >1 s).
- `depth()` (516) → HxW float32 metres, NaN = no return, read-only, or None.
- `camera()` (522) → `{width, height, K (3x3), frame_id, world_T_camera:
  {translation, rotation_xyzw}}` from `CameraStreams.get_camera()` +
  `extrinsics_from_yaml`.
- `deproject(u, v, z)` (560) → `{"x","y","z"}` world point via K and
  world_T_camera; ndarray inputs → Nx3.
- `frame_stamps()` (541) → `{rgb_wall_ns, depth_wall_ns, age_ms}`; age_ms is
  from the OLDEST present frame (conservative) to the snapshot moment.
- Gating: on the scalar lanes every accessor raises a ValueError naming
  rate_hz. All five are noted in `channels_read`.

Snapshot semantics (design decision): `_begin_camera_tick()` (491) clears the
snapshot before each check call; the FIRST accessor inside the call takes it
lazily via `CameraStreams.array_snapshot()` (image_buffer.py:751, one lock
acquisition for a coherent rgb+depth pair, arrays decoded fresh with
`writeable=False`). Lazy-inside-the-call is deliberate: it keeps "every
accessor in one call returns the same frames" AND puts the snapshot cost
inside the call's wall budget, which is what lets the wall-strike test inject
a slow stream.

`np` is PROVIDED (not imported) via `compile_check(source, extra_namespace)`
— only on the array lane; the scalar namespace is unchanged and `import`
stays forbidden.

`_json_safe` now summarises ndarrays (`_ndarray_summary`, 328):
`{"ndarray": [shape], "dtype", "min", "max", "nan_frac"}` — so
reference_values and every payload stay array-free.
`_state_ndarray_bytes` (386) backs the 32 MB `ctx.state` cap
(`ARRAY_STATE_MAX_BYTES`, 39), enforced in `evaluate()` (922) as
`AuthoredReturnError` → `authored_bad_return` with a clear detail.

## 2. The array lane (engine.py)

- Third daemon thread (`_array_worker`, 238), started/stopped with the
  existing `start_authored_lane`/`stop_authored_lane`, selecting
  `feed == "authored_array"` (set by AuthoredMonitor when rate_hz is given) —
  so neither `_evaluate_feed` nor the scalar authored worker ever touches an
  array monitor, and a slow array monitor delays only this lane.
- Scheduling: per-monitor `next_due_ns`; the lane sleeps (stop-aware, ≤0.2 s)
  to the next due monitor; the next tick is scheduled from NOW after each
  call, so a slow call skips ticks instead of accumulating debt.
- Budgets, both enforced per call: the existing 20000-line budget (Python
  pixel loops) and `ARRAY_WALL_BUDGET_MS = 150` (32) measured around
  check(ctx) — numpy C work is invisible to the tracer, so the wall clock is
  the bound. A wall overrun is a strike through the SAME `_authored_strike`
  mechanism and limit (5 → `authored_timeout`, detail naming the budget and
  measured ms); it never kills the thread. `monitor.last_wall_ms` and
  `monitor.calls` record per-call cost.
- `rate_hz` is clamped to (0, 2.0] server-side (`_clamp_rate_hz`,
  watchdog_mcp_server.py:1002); the arm result reports `rate_hz`,
  `lane: "array"`, and `rate_hz_clamped`/`rate_hz_requested` when clamping
  happened; rate_hz ≤ 0 is an error.

## 3. purpose (server + schema)

- `arm_custom_monitor(..., rate_hz, purpose, threshold_rationale,
  halt_on_fire)` (lines ~1583+): purpose "clause" (default, exactly the old
  behaviour) or "observation". Observation: `halt_on_fire` refused; clause may
  be null; requires a non-empty `threshold_rationale`; provenance is
  overridden after resolution to `threshold_source="agent_chosen"`,
  `threshold_verified=False` (registry field/value nulled) — in
  `_prepare_authored_monitor`, so the dry run shows the same.
- Schema: `MonitorArmedPayload` and `MonitorFirePayload` gain
  `purpose: str = "clause"` and `threshold_rationale: str | None = None`
  (optional-field pattern; every existing log parses). Carried by
  `engine._authored_fields`, so they appear on arm/fire events and the arm
  result for every authored monitor.
- Finding bookkeeping verified: `_note_finding` is called ONLY from
  record_verdict (fires never touch `findings_during_run`/`recovered`), so no
  exclusion was needed — an observation fire wakes `wait_for_event` like any
  fire (engine `_handle_fire` unchanged) and never becomes a finding.
  Pinned by `test_observation_arm_and_fire_carry_purpose_and_agent_chosen`.
- `halt_on_fire` is NEW on arm_custom_monitor (it previously wasn't
  exposed); default False keeps prior behaviour, and observation refuses it.

## 4. Docstrings

- `arm_custom_monitor`: the camera accessor block with exact shapes, `np`,
  snapshot semantics, None cases, 1.0 s staleness, the 2 Hz cap, 150 ms wall
  budget, 32 MB state cap, purpose/threshold_rationale rules, and one generic
  array-lane observation example (depth change inside a ctx.state-stored ROI
  against `ctx.value("change_m")` — no scene-specific numbers, not an
  intrusion detector).
- `dry_run_custom_monitor`: rate_hz spaces ticks at 1/rate_hz (still clamped
  to `_DRY_RUN_MAX_TICKS`), fresh camera snapshot per tick, `wall_budget_ms`
  reported (150 vs 20); purpose accepted for symmetry and echoed.
- Event catalog: `LookPayload`'s description now states look wakes
  wait_for_event only when "look" is named in event_types;
  `MonitorFirePayload`'s states that it carries purpose. (Both surface
  through `available_event_types`, which is generated from these docstrings.)

## 5. Stage 1 leftovers

a. Depth (and pointcloud) stream stats now record the receiving topic
   (`_on_depth`/`_on_cloud` take the topic via functools.partial at attach).
b. `look()`'s text part carries the full `streams` block only when a view
   failed; a clean look carries `streams_ok: {rgb_hz_est, depth_hz_est,
   camera_info_topic}` (look.py `_streams_ok`, 178 — rates estimated as
   received/elapsed-since-attach).

## 6. Tests and results

New: `test_array_lane.py` (13) — depth-change fire and not-before +
channels_read; snapshot identity within one call + read-only frames + one
snapshot per call; None frames + camera shape; deproject scalar/vectorised;
frame_stamps; scalar-lane camera accessor raises naming rate_hz; np provided
only on the array lane; full-frame Python loop hits the line budget; state
cap → AuthoredReturnError; json_safe ndarray summary; reference_values
summarise a stored frame; wall-budget strikes (200 ms fake stream) →
authored_timeout with the wall detail while a TF-lane library monitor fires
mid-sleep; array fire through `_handle_fire` with purpose/rationale on the
events and the wait woken.

New: `test_authored_purpose_and_rate.py` (10) — server-level against a fake
context: rationale required, halt_on_fire refused, bad purpose rejected,
observation arm+fire provenance and NO finding, clause purpose keeps registry
provenance, rate clamp reported, rate 0 errors, rate_hz=None keeps the
pre-Stage-2 result surface (and feed "authored"), dry run with rate (500 ms
ticks, wall_budget_ms=150, arms nothing, no events), dry run without rate
(50 ms, 20 ms budget). (This file also restores server-level arm/dry-run
coverage lost when the pre-pull `test_authored_dry_run.py` stash was
dropped.)

Updated: `test_authored_monitor.py`'s ctx public-surface list (+rgb, depth,
camera, deproject, frame_stamps).

Full run — STAGE1_VERIFY §2 targeted list + validator + skill-event coverage
+ camera receive tests + both new files:

```
235 passed in 10.88s   (zero failures)
```

## Unresolved / notes

- "Byte-identical outputs" for rate_hz=None is interpreted as: all
  pre-existing keys and values unchanged. The arm result and payloads
  necessarily gain `purpose` (and `threshold_rationale: null`) because the
  same task mandates carrying purpose on the payloads. The compat test pins
  the pre-existing surface.
- The arm-time first call is NOT under the 32 MB state cap (the cap is
  enforced per evaluate()); a first call that stores an oversized array is
  caught on the first lane tick.
- `ARRAY_RATE_MAX_HZ` (2.0) vs the RGB forward throttle (5 Hz) and camera
  rate (~14.5 Hz): at 2 Hz a monitor sees every-other forwarded frame at
  worst; fine for observation triggers.
- Rebuild required before a live run: `colcon build --packages-select
  robot_backend`.
