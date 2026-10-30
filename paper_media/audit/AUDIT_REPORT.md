# WatchDog data integrity audit

Diagnosis only. No code was changed. Written 2026-10-29 from a code read and from
session data already inspected in this project. Paths are relative to the repo root;
`rb/` means `ros2_ws/src/robot_backend/robot_backend/`.

Evidence labels:
- **verified**: I read the cited lines myself.
- **per code read**: cited by a read-only search pass over the file; not re-read line by line.
- **inferred**: a conclusion the code or data supports but does not prove. Each one names what would confirm it.

## Results of the first audit run (2026-10-29, all 112 session directories)

Run with `session_audit` over `agents/transcripts/sessions`, bag checks on. Full output is in `paper_media/audit/out/report.json`.

| Check | ok | warn | corrupt |
|---|---|---|---|
| foreign_mcp_records | 98 | 0 | 14 |
| role_separation | 112 | 0 | 0 |
| guard_mode, json_parse, time_format, time_out_of_order, look_paths, bag | 112 | 0 | 0 |
| registry_provenance | 100 | 12 | 0 |
| late_file_writes, time_outside_span | 102 | 10 | 0 |
| fires | 71 | 41 | 0 |

- **Foreign records, 14 sessions.**
  - 12 are scored S1 or S2 runs. One is the voided S1 run 20261007T205057Z. One more, 20261016T030548Z, was not in the earlier list.
  - Every WatchDog-server block was written after its run ended, in blocks of exactly 71 calls: the test-run signature.
  - Two single records are stale `begin_step` calls from executor `scene` servers that outlived an earlier attempt. Each blocked about 125 s on a gate, then logged into the next session once the pointer had moved: 20261008T025202Z and 20261016T030548Z. This confirms mechanism 2 in Q2 with data.
  - No foreign call falls inside any run, so `role_separation` is clean in all 112 sessions.
- **Bag checks pass everywhere.** All 112 bags open, carry `/tf`, and carry `finger_joint` effort on `/joint_states`.
- **The 12 provenance warnings are the 12 Class 4 sessions.** None can establish both roles' `max_clamp_effort` from the session alone, as predicted in Q5.
- **Fires.** In all 82 sessions that contain fires, every fire has `action_taken: flag`. Two are never delivered: both S2 `20261015T030950Z`, fired after the WatchDog's last wait. The remaining fire warnings are deliveries slower than 10 s. These are latency, not loss.
- **Expected non-findings.** The two REF sessions have no label or WatchDog config by design. The voided S3_opus_medium session has no `run_ended`.
- **Scores.** None of these change a reported score. The WatchDog scorer ignores call logs. The S4 scorer, which does not filter, never reads these sessions. The analysis pipeline filters by server and process id.

## Bottom line

1. **The cross-session contamination has a confirmed mechanism.** The call log re-reads `~/.watchdog/current_session` on every write. Any process that has a task-path environment variable set appends into whatever session the pointer last named. The foreign records in 12 sessions have the signature of a test suite run, not a live agent.
2. **No paper number computed so far is affected by that contamination.** The WatchDog scorer never uses the call logs. The only scorer that does is the S4 scorer, and none of the 12 contaminated sessions is S4. The analysis pipeline filters by server and process id.
3. **The session manifest has never been written.** The code runs at every bringup but fails silently: it loads every mesh with trimesh, which the launch interpreter lacks (inferred). Two safety guards depend on the manifest and are therefore inert.
4. **For Class 4, a session alone cannot prove the WatchDog's `max_clamp_effort`.** There is no WatchDog provenance file. The executor's provenance stores a path and a hash, not the registry content.
5. **The WatchDog server registers `halt_execution` and `abort_run`.** This conflicts with the stated invariant "WatchDog holds zero command tools", depending on whether a stop counts as a command. No session ever called either tool.

---

## 1. Answers to Part A

### Q1. Session directory resolution

Shared rule, `rb/watchdog/session.py:129-142` (verified): `WATCHDOG_SESSION_DIR`, else the path in `~/.watchdog/current_session` (read from disk on each call, `:103-112`), else a fresh timestamp. `active_session_dir()` returns None instead of a fresh path (`:145-154`).

| Writer | How the directory is chosen | When |
|---|---|---|
| Bringup (`sim_moveit_bringup.launch.py`) | `session_dir` launch arg, else `WATCHDOG_SESSION_DIR`, else fresh (`:67-83`, verified). Never reads the pointer. Writes the pointer (`:243-248`) and sets `WATCHDOG_SESSION_DIR` for its own children (`:270`). | Once, at bringup |
| Orchestrator | `log_dir` launch arg set to `<session>/orchestrator` (`launch.py:261-262`, verified). File opened once in `__init__` (`cell_orchestrator/node.py:168-174`, per code read). | Once |
| rosbag | `ros2 bag record -o <session>/bag` (`launch.py:403-407`, per code read) | Once |
| Executor servers (motion, scene, gripper) `events_<server>.jsonl` | `WATCHDOG_EVENT_LOG` env, else `event_log_default()` = active session (`session.py:164-174`, verified). The EventBus opens the file once (`rb/events/bus.py:104-108`, verified). | Once, at server start |
| S4 executor_monitor `events_executor_monitor.jsonl` | `EXECUTOR_MONITOR_EVENT_LOG` env, else active session (`executor_monitor_mcp_server.py:152-160`, per code read) | Once |
| WatchDog server `events_watchdog.jsonl`, frames, authored monitor sources | `WATCHDOG_MONITOR_LOG` env, else `monitor_log_default()` (`session.py:177-184`, verified). `ctx.session_dir` fixed in `_ensure_watchdog` (`watchdog_mcp_server.py:681-698`, per code read). | Once |
| Monitor engine | No path of its own; publishes through its host server's bus (`watchdog_mcp_server.py:718-723`, per code read) | Same as host |
| Event recorder (`rb/watchdog/recorder.py`) | Writes no file; in-memory buffers only (per code read) | n/a |
| **Every server's `mcp_calls_<role>.jsonl`** | `_call_log_path()` takes the role from the task-path env var and the directory from `active_session_dir()` (`rb/mcp_servers/role_context.py:316-326`, verified) | **On every record** (`role_context.py:386`, verified) |
| Agent launchers | `run_executor.sh:65` and `run_watchdog.sh:49` read the pointer only to write `agent_config_<role>.json`. They pass no directory to `claude` (per code read). The mcp.json files deliberately omit `WATCHDOG_SESSION_DIR` (`executor.mcp.json:12-17`, verified). | Once |

So every server's event log is pinned to the session that was active when the server started. Its call log, however, follows the pointer as it moves.

### Q2. Cross-session contamination, ranked by likelihood

**1. Confirmed mechanism, inferred source: a process outside any run, with a task-path env var set, while the pointer still names the last session.**
- `_call_log_path()` resolves the session on each write (`role_context.py:316-326, 386`, verified).
- The pointer persists after a bringup exits. Nothing clears it (`session.py:115-126` only writes it).
- The role comes only from which task-path variable is set (`role_context.py:70-94`, verified). A WatchDog-server tool called in a process with `EXECUTOR_TASK_PATH` set therefore logs into `mcp_calls_executor.jsonl`, which is exactly what was found.

The foreign records look like a test run:

| Evidence | Where |
|---|---|
| Record content: `record_run_verdict` called three times in 2 ms with `explanation: "rollup"`, `steps_covered: ["place_u_brown"]`, `subscriber_count: 0`, `seq: 0`, elapsed 0.06 to 0.3 ms | `20261007T220959Z/mcp_calls_executor.jsonl` lines 93-95 |
| Exact multiples of one block: 71, 142, 213 records per session | foreign-record scan done earlier in this project |
| Timing: the run ended 22:18:32 (line 92). The foreign block starts 22:27:02 (line 93). The next run is 20261007T223412Z. This sits in the gap between runs, when the batch notes record test-suite runs ("Suite 627/7/0", `paper_results/batches/S1/batch.yaml:12`). | session data |
| One block lands 4.8 days after the last S1 run, just before the first S2 run (20261015T003820Z), i.e. a test run before starting S2 that wrote into S1's last session | 20261010T023928Z |

`seq: 0` means the event was never published through a configured EventBus (`bus.py:184-189` stamps seq at 1 or more). So the test traffic reached the call log only, not the events files (inferred).

To confirm, run `grep -rn '"rollup"' ros2_ws/src/robot_backend/test agents/tests` and check that those tests set `EXECUTOR_TASK_PATH` without setting `WATCHDOG_SESSION_DIR` to a temporary directory.

**2. Possible by code, not seen in data: an old MCP server outliving its run.** A server exits when its Claude CLI closes stdio (inferred). Nothing kills it or checks for it (Q3). If it survives into the next bringup, its call log follows the new pointer, but its events file stays in the old session (Q1). That would split one process across two sessions.

**3. Possible: a stale `WATCHDOG_SESSION_DIR` export.** The env var beats the pointer (`session.py:136-138`). The bringup's own log tells the operator to "Export the printed value into the shell that launches the agents" (`launch.py:265-269`, verified). This contradicts the mcp.json notes, which say never to export it. A shell that keeps an old export would send a new run's servers into the old directory.

**4. Low: duplicate servers from editor windows.** This was the 2026-08-31 incident. `.mcp.json` now holds only `kit` (`.mcp.json:33-42`, verified).

**5. Not a cause: reused log handles.** Event logs are opened once per process. Call logs are opened per write, but at a re-resolved path.

### Q3. Process teardown

- Each server's `main()` wraps `mcp.run()` in `try/finally` shutdown and registers `atexit` (watchdog `:3121-3124`, `:807`; per code read).
- The orchestrator closes its log in `destroy_node` (`node.py:607-626`, per code read).
- The bag stops on Ctrl-C of the bringup (`docs/dev/BATCH_RUNBOOK.md:105`).

Nothing kills MCP servers. There is no `pkill`, no process or PID check, and no "already running" check in the files read. The only start guard is `assert_session_dir_unused`, which looks only for `manifest.json` in the target directory (`session.py:279-287`, verified). The manifest is never written (Q4) and every bringup mints a fresh directory, so this guard can never trip.

A WatchDog server or monitor engine can therefore survive into the next run if its CLI does not exit (inferred). `atexit` does not run on SIGKILL.

### Q4. Session manifest

- **The code exists and runs.** `_write_manifest` is called unconditionally while the launch actions are built (`launch.py:280`, verified). It calls `sess.write_manifest(...)` (`:127-140`, verified).
- **Why it fails.** `write_manifest` calls `registry.mesh_provenance()` before writing anything (`session.py:333-337`, verified). That loads every mesh object through `load_mesh_geometry`, which does `import trimesh` (`rb/registry/geometry.py:22-39`, verified). The ImportError is re-raised as a ValueError.
- **Where the failure disappears.** The launch catches it at `launch.py:148-149` (verified) and only emits `LogInfo("Session manifest write failed: ...")`. The pointer write in the same block succeeds (`:243-246`), so `robot_backend` itself imports fine.
- **Inferred cause.** trimesh is absent from the system ROS Python that runs the launch file. It exists only in `ros2_ws/.venv`, which the MCP servers re-exec into. To confirm, run `grep -r "Session manifest" ~/.ros/log/ | head` and `/usr/bin/python3 -c "import trimesh"`.
- **Knock-on effects.**
  - The `RegistryMismatchError` guard (`session.py:221-262`) never runs.
  - The session reuse guard (Q3) never trips.

Per-session metadata files that are actually written:

| File | Writer |
|---|---|
| `agent_config_executor.json` | `run_executor.sh:67-111`. Includes the registry override path and sha (`:78-84`). Per code read. |
| `agent_config_watchdog.json` | `run_watchdog.sh:51-68`. No registry field. Per code read. |
| `registry_provenance_executor.json` | `rb/registry/loader.py:87-103`, only when `EXECUTOR_REGISTRY_PATH` is set (per code read) |
| `ground_truth_injections.jsonl` | `isaac/script_editor/inject_intruder.py:552-558` (per code read) |
| `intruder_leak_audit.json` | `scripts/audit_intruder_leak.py:348`, run by hand after the session |
| `claude_cli_version.txt`, `*.settings.json` copies | No code. These are manual steps in `docs/dev/RUN.md:189-191`. |
| `authored_monitors/*.py`, `frames/...` | WatchDog and executor_monitor servers, scene server |

### Q5. Registry provenance

- **Executor.** `registry_provenance_executor.json` holds `role`, `registry_path`, `registry_sha256` and `override_env` (data: `20261010T020201Z`).
  - It is written each time `executor_registry_override()` runs in an executor-role process with the override set: at gripper and scene init, and on every `get_rendered_task` (per code read).
  - It goes to `active_session_dir()`, re-resolved each time, so it can land in whichever session the pointer names (inferred, same mechanism as Q2).
- **WatchDog.** No equivalent file exists.
  - The WatchDog loads the install-space registry: package share `config/model_registry.yaml`, with a source fallback (`rb/watchdog/recorder.py:149-162`, per code read).
  - The override is ignored for the WatchDog role (`loader.py:64-75`, per code read).
- **Can a Class 4 session prove both values? No.**
  - The executor's value can be recovered only while the file at `~/.watchdog/registry/model_registry.yaml` still matches the recorded sha. The session stores the path and hash, not the content, and that path is reused for every corruption.
  - The WatchDog's value is recorded only in `batch.yaml` prose (for example `S1/batch.yaml:386-388`: install sha `32f79017...`, `u_brown` 10).

### Q6. Scenario, fault class, assembly and trial at scoring time

- **WatchDog and S4 scorers.** `scenario`, `class` and `task` come from the label YAML only (`watchdog_scorer.py:464`, per code read). The label is a CLI argument, or the only `*.yaml` in the session directory (`watchdog_scorer.py:1454-1462`, `s4_scorer.py:516-522`, per code read). Any single YAML file in the directory is accepted, not just `gt_*.yaml`.
- **run_id.** From the label, falling back to the directory name (`:463`). The observation scorer always uses the directory name, and takes the scenario from its `--scenario` CLI flag, default empty (`observation_scorer.py:357, 562`, per code read).
- **Assembly and trial.** Not recorded in any session file except the `"assembly": "2"` field in `agent_config_*.json`. Trial exists only in run names and `batch.yaml` notes.
- **Not read by any scorer:** `batch.yaml`, `agent_config_*.json`.
- **Join by path.** The `analysis/` pipeline reads model and effort from `batch.yaml`, and joins rows by the session timestamp found in `label_used`, which is a directory path.
- **Consistency checks.** `scenario_consistency_warning` compares the label's scenario with arm origins for S1, S3 and S4 only (`watchdog_scorer.py:496-506`, per code read). Nothing checks the label's `run_id` against the directory, or whether the S4 scorer is run on a non-S4 label.

### Q7. Scorer exposure to foreign records

- **watchdog_scorer.py** loads the call log (`:189`) but never uses it after `:446` (per code read). Its metrics come from event files only, filtered by `event_type`.
  - It takes the last `run_verdict` (`:509`), so a foreign `run_verdict` in `events_watchdog.jsonl` would change the final verdict.
  - That has not been observed. The test traffic did not publish events (Q2).
- **s4_scorer.py `serialization_metrics`** reads `mcp_calls_executor.jsonl` with no server, process id or upper time filter (`:376-401`, verified). A call counts as "monitor" by tool name alone (`:379-385`, verified).
  - Foreign WatchDog records after the first `begin_step` would inflate `monitor_calls_in_run`, `monitor_tool_wall_sec` and `interleaved_monitor_blocks`.
  - They would also change the fire-to-adjudication metrics, through `ADJUDICATION_TOOLS` (`:110`, per code read).
- **observation_scorer.py** never reads call logs (per code read).
- **Effect on existing numbers: none known.** The 12 contaminated sessions are all S1 or S2. The analysis pipeline filters by server and dominant process id.

### Q8. Timing

- **Producers.** Every `wall_time_iso` comes from the same expression, `datetime.now(timezone.utc).isoformat()`, giving microseconds with a `+00:00` suffix:
  - events, `rb/events/schema.py:733` (per code read)
  - calls, `role_context.py:415` (verified)
  - launchers, `run_*.sh` (per code read)
- **Format edge case.** Python omits the fraction when microseconds are exactly 0. String sorts in the scorers (`s4_scorer.py:172, 377`, `observation_scorer.py:261`, `watchdog_scorer.py:229, 602`, per code read) still order correctly, because `+` sorts before `.`.
- **`timestamp_ns`.** It is `time.monotonic_ns()` (`schema.py:732`, per code read). No scorer, compositor or analysis code uses it across processes. `recorder.py:509-513` refuses it explicitly. Simulation time is never used.
- **First event of each process.** `make_event` runs before the one-time subscriber wait (`bus.py:225` vs `:175`, verified), so that event's `wall_time_iso` can be up to 2.0 s earlier than its actual publish (`bus.py:65`).
- **Envelope time instead of `trigger_wall_ns`.** The S4 scorer times fire to adjudication from the fire envelope's `wall_time_iso` (`s4_scorer.py:442-446`, per code read). The difference was measured at about 0.1 ms in session 20261019T204942Z (trigger `...115960677746`, envelope `20:51:55.960784`), so it is negligible today.

### Q9. S4 guard mode

- The orchestrator writes `gate_unguarded_by_config` whenever its `guard_mode` parameter is `unguarded` (`node.py:410-421`, verified).
- It writes `gate_unguarded_latched` only in guarded mode, when no monitor signalled before the first gate timed out (`:426-440`, `:501-521`, verified).
- `guard_mode` comes only from the bringup launch argument, default `guarded`. The run scripts do not set it; the operator types it (per code read: `run_executor.sh:17-21` is a comment only).
- **S4 isolation rests on two things only:** the S4 executor config, which lists motion, gripper, scene and executor_monitor, and `--strict-mcp-config`. Nothing stops a concurrent `run_watchdog.sh`.
- **The S4 scorer checks only part of this.** `check_validity` inspects only `verdict`, `run_verdict`, `monitor_armed` and `monitor_fire` in `events_watchdog.jsonl` (`s4_scorer.py:151-153`, per code read). It ignores `mcp_calls_watchdog.jsonl` and `server: watchdog` records inside the executor log.
- **Data.** All 28 S4 and S4_opus_medium sessions have only `gate_unguarded_by_config` gate records (476 in total), no latched record, and no WatchDog log files (orchestrator inventory done earlier in this project).

### Q10. Fire records

- **Possible values.** `action_taken` is `flag` by default. When `halt_on_fire` is set and a halt client exists it becomes `halt`, `halt_no_active_goal` or `halt_failed`, and `halt_result` carries the halt dict (`rb/watchdog/engine.py:494-514`, verified).
- **There is no global observe-and-record mode.** Halting is chosen per monitor by the arming agent: `halt_on_fire=False` is the default on `arm_monitor` and `arm_custom_monitor`, and it is refused for observation monitors (per code read).
- **Data.** All 408 recorded fires have `action_taken: flag` and `halt_result: null`.
- **Invariant tension.** The WatchDog server registers `halt_execution`, which cancels MoveGroup goals (`watchdog_mcp_server.py:2683-2694`, verified), and `abort_run`, which makes every later gate refuse (`:2696-2717`, verified). No session ever called either tool.

### Q11. look() frame paths

- **What is stored.** look() writes `<session_dir>/frames/{look_|exec_look_}{seq:04d}_rgb.png` and stores `str(path)`, an absolute, unresolved path, in the event payload (`rb/vision/look.py:394, 458, 276-277`, per code read).
- **Where `session_dir` comes from.** `active_session_dir()`, which is absolute because the pointer is absolute.
- **The real risk.** In this repository the paths name this machine. They break when sessions are copied or moved.
- **Overwrite risk (inferred).** The sequence counter is per process (`look.py:55-63`), so a server restarted inside one session would overwrite `look_0001_*`.
- **Recommendation.** Record a session-relative path (`frames/look_0001_rgb.png`) alongside the absolute one. Readers already fall back to the base name: the compositor does, at `paper_media/replay/load_session.py` `resolve_session_image`.

### Q12. Other risks found

- **Gate keys never cleared.** `_get_gate_event` creates an event per `(step_id, phase)` and nothing resets it (`node.py:262-269`, verified; no reset in `:260-560`). A repeated step id passes its gate immediately, before any re-arming. The audit script flags repeated step ids.
- **Subscriber wait satisfied by the bag recorder.** The first-publish wait returns when any subscriber matches (`bus.py:146-149`, verified), and the bag recorder subscribes to `/watchdog/events` (`launch.py:24`). A `begin_step` can therefore be lost to the WatchDog under volatile QoS (inferred). The envelope's `subscriber_count` also counts the bag.
- **`watchdog_ready` published even when the service call failed.** `ready.py:85-86` publishes in `finally` (per code read), so the timeline can show readiness the orchestrator never received.
- **Swallowed logger exceptions.** All of these drop records silently, with no counter:
  - event file write, `bus.py:203-204` (verified)
  - call-log write, `role_context.py:398-401` (verified)
  - orchestrator log, `node.py:259-260` (verified)
  - malformed envelopes in the recorder, `recorder.py:496-497` (per code read)
- **`seq` incremented without a lock** while several threads publish (`bus.py:184`, verified). Duplicate `seq` values are possible. Ordering uses `wall_time_iso`, so the impact is cosmetic.
- **Flushing.** Event files are line buffered, with no fsync. Daemon threads that publish after `close()` drop their writes silently (`bus.py:210-213`). A SIGKILL skips `atexit`.
- **Label pick-up.** The scorer accepts any single `*.yaml` in the session (`watchdog_scorer.py:1454-1462`). An extra YAML file makes it fail loudly. A mislabelled one is used silently.
- **Double halt on near-simultaneous fires.** Two fires of one monitor can each call `halt()` before the armed check (`engine.py:496` vs `:545-548`), producing two halt events (inferred, per code read). Not reachable in current data, because no monitor used `halt_on_fire`.

---

## 2. Ranked findings

| # | Severity | Finding | Evidence | Proposed fix | Touches an invariant? |
|---|---|---|---|---|---|
| 1 | Corrupts results (latent; contaminated 12 sessions, no score yet) | The call log re-resolves the session pointer on every record. Out-of-run processes write into the last session. | `role_context.py:316-326, 386`; records in `20261007T220959Z/mcp_calls_executor.jsonl:93-95` | Resolve the call-log path once per process. Tests set `WATCHDOG_SESSION_DIR` to a temporary directory. Clear or timestamp the pointer when a bringup exits. | No |
| 2 | Corrupts results if contamination reaches S4 | S4 serialization metrics do not filter the call log by server, process id or run window | `s4_scorer.py:376-401` | Filter by the executor server set, dominant process id per server, and `[first begin_step, run_ended]`. Rescore and diff against the current scored CSVs; the scored batches must not change. | No |
| 3 | Corrupts provenance (results unverifiable) | The manifest is never written: the trimesh import fails inside `mesh_provenance()` and the error is swallowed | `session.py:333-337`, `geometry.py:22-39`, `launch.py:148-149` | Make mesh provenance optional inside `write_manifest`, recording the error as a field, and fail the bringup for scored runs if `manifest.json` is absent | No |
| 4 | Corrupts provenance | Both guards that depend on the manifest are inert: registry mismatch and session reuse | `session.py:221-287` | Fixed by #3. Also make the reuse check look at any `events_*.jsonl` or `mcp_calls_*.jsonl`, not only the manifest. | No |
| 5 | Corrupts Class 4 claims | No WatchDog registry provenance. The executor provenance stores a path and hash, not the content. | `loader.py:87-103`; `recorder.py:149-162` | Section 4 design: write both provenance files once at startup and copy the registry bytes into the session | Supports the Class 4 invariant; does not change it |
| 6 | Corrupts results (unknown extent) | Gate events are never cleared, so a repeated step id passes its gate unwatched | `node.py:262-269` | Reset the event when a request consumes it, or key gates by a per-request token. Run the audit first to learn whether any session repeats a step id. | Gate keys are the executor's step labels. The fix changes the orchestrator only, not WatchDog arming. Confirm before changing. |
| 7 | Corrupts results (unknown extent) | A `begin_step` can be lost because the subscriber wait counts the bag recorder | `bus.py:146-149`, `launch.py:24` | Wait for a subscriber other than the bag, by node name via `get_subscriptions_info_by_topic` | No. WatchDog arming stays anchored on its own instruction plus observed events. |
| 8 | Needs a decision | The WatchDog holds `halt_execution` and `abort_run` | `watchdog_mcp_server.py:2683-2717` | Either define "command tool" as motion-producing and record that stop and abort are allowed, or gate both tools behind a scenario flag | **Yes**: "WatchDog holds zero command tools" |
| 9 | Corrupts results if it happens | S4 isolation is enforced only by config. The scorer's check misses WatchDog call logs and other event types. | `s4_scorer.py:151-153`; `run_executor.sh:124` | S4 validity also fails on `mcp_calls_watchdog.jsonl`, any `events_watchdog.jsonl`, or `server: watchdog` executor records. Bringup records `guard_mode` in the manifest. | Supports the S4 invariant |
| 10 | Corrupts results if it happens | The bringup log tells the operator to export `WATCHDOG_SESSION_DIR`, contradicting the configs. A stale export redirects a run. | `launch.py:265-269` vs `executor.mcp.json:12-17` | Reword the log line. Launchers refuse to start if `WATCHDOG_SESSION_DIR` is exported and differs from the pointer. | No |
| 11 | Corrupts provenance | `registry_provenance_executor.json` is rewritten on each override call, into the pointer-named session | `loader.py:87-103` (per code read) | Write once at process start into the pinned session, as in #1 | No |
| 12 | Corrupts video or figures | look() paths are absolute. Frames are overwritten if a server restarts inside one session. | `look.py:55-63, 276-277, 394, 458` | Add a session-relative path. Start the sequence after the highest existing index. | No |
| 13 | Corrupts video or figures | The first event of each process is stamped up to 2 s before it is published | `bus.py:175, 225` | Stamp `wall_time_iso` at publish, or record both stamps | Keeps `wall_time_iso` as the ordering field |
| 14 | Cosmetic, but hides loss | Swallowed logger exceptions, no loss counter | `bus.py:203-204`, `role_context.py:398-401`, `node.py:259-260` | Count failures per process and write the count at shutdown | No |
| 15 | Cosmetic | `watchdog_ready` is published even when the service call failed | `ready.py:85-86` (per code read) | Add `ok` to the payload | No |
| 16 | Cosmetic | `seq` race in EventBus | `bus.py:184` | Increment under a lock | No |
| 17 | Cosmetic | The scorer accepts any single `*.yaml` as the label | `watchdog_scorer.py:1454-1462` | Accept only `gt_*.yaml` | No |

---

## 3. What is affected, and what only the script can answer

### Affected, as far as the code review and earlier data checks show

- **Foreign WatchDog test records in the executor call log**, 71, 142 or 213 records each:
  - S1 (9 sessions): 20261007T220959Z, 20261007T223412Z, 20261007T232107Z, 20261007T234346Z, 20261008T025202Z, 20261008T032957Z, 20261008T195453Z, 20261010T014653Z, 20261010T023928Z
  - S2 (3 sessions): 20261015T003820Z, 20261015T011118Z, 20261015T203118Z
- **Scores in those sessions are unaffected.** The WatchDog scorer ignores call logs, and the analysis pipeline drops these records. Their raw `mcp_calls_executor.jsonl` files are not clean, and any future tool that reads them unfiltered would be wrong.
- **All 112 session directories lack `manifest.json`.** Registry identity for every run rests on `batch.yaml`.
- **Class 4 sessions in S1 to S4** cannot prove the WatchDog's `max_clamp_effort` from the session alone. The executor side is provable only while `~/.watchdog/registry/model_registry.yaml` still matches the recorded sha.
- **S4 guard records are clean in all 28 S4 and S4_opus_medium sessions:** `unguarded_by_config` only, no WatchDog logs.
- **Fires.** All 408 have `action_taken: flag`. No halt happened.

### Only the audit script can answer

- Whether any session repeats a step id, so that a gate passed unwatched (finding 6)
- Which other session's process wrote each foreign block, matched by process id across all sessions
- Whether files were modified after their run ended (test traffic writing provenance or call records)
- Out-of-order or out-of-span records, large gaps, and malformed timestamps
- `begin_step` / `end_step` mismatches, verdicts naming steps never begun, and steps without a step verdict
- Fires never delivered to the adjudicating role, and late deliveries
- Whether each executor provenance file still matches its recorded sha, and whether Class 4 values can be established
- look() paths that no longer resolve
- Bag checks: the bag opens, `finger_joint` effort is present, `/tf` is present, and the bag span overlaps the run

Not answerable by the script: whether a `begin_step` was dropped before the WatchDog subscribed (finding 7). The event is still in `events_scene.jsonl` and the bag, so its absence on the WatchDog side leaves no record.

---

## 4. Proposed design (not implemented)

Both files are written once, at session start, atomically (temp file then rename). Each writer resolves the session directory once and never rewrites the file. Neither depends on trimesh.

### `manifest.json`, written by the bringup before the pointer

```json
{
  "schema_version": 2,
  "session_id": "20261029T120000Z",
  "created_wall_time_iso": "2026-10-29T12:00:00.123456+00:00",
  "created_by": {"process": "sim_moveit_bringup", "pid": 12345, "host": "MohammadNsr", "user": "mohammad", "python": "/usr/bin/python3"},
  "session_dir_rule": "fresh | arg | env",
  "pointer": {"path": "~/.watchdog/current_session", "written_wall_time_iso": "..."},
  "declared": {
    "batch_id": "S5", "scenario": "S1 | S2 | S3 | S4", "fault_class": 1,
    "assembly": "fmb_assembly_2", "trial": 3, "run_label": "fmb2_s1c1_run03",
    "source": "launch arguments typed by the operator"
  },
  "launch": {"guard_mode": "guarded | unguarded", "gate_timeout_sec": 120.0, "record_session": true,
             "record_images": false, "record_pointcloud": false, "bag_topics": ["/joint_states", "/tf", "/tf_static", "/watchdog/events"],
             "all_arguments": {}},
  "git": {"commit": "...", "dirty": false, "diff_sha256": null},
  "registry": {
    "install_path": ".../install/robot_backend/share/robot_backend/config/model_registry.yaml",
    "install_sha256": "...",
    "source_path": ".../src/robot_backend/config/model_registry.yaml",
    "source_sha256": "...",
    "install_matches_source": true,
    "copy": "registry/model_registry.cell.yaml"
  },
  "roles_expected": {
    "executor": {"servers": ["motion", "scene", "gripper"], "mcp_config": "executor_2.mcp.json"},
    "watchdog": {"servers": ["watchdog"], "mcp_config": "watchdog_2_s3.mcp.json"}
  },
  "clock": {"ordering_field": "wall_time_iso", "per_process_only": ["timestamp_ns"], "never_use": ["header.stamp"]},
  "mesh_provenance": {"status": "ok | unavailable", "error": null, "objects": {}}
}
```

`declared` must come from new launch arguments typed by the operator, never from directory or file names. For S4, `roles_expected.watchdog` is `null`, and `guard_mode` must be `unguarded`.

### `registry_provenance_watchdog.json`, written by the WatchDog server in `_ensure_watchdog`

```json
{
  "role": "watchdog",
  "written_wall_time_iso": "...",
  "pid": 23456,
  "server": "watchdog",
  "session_id": "20261029T120000Z",
  "session_dir_rule": "env | pointer",
  "registry_path": ".../install/robot_backend/share/robot_backend/config/model_registry.yaml",
  "registry_source": "install_share | source_fallback",
  "registry_sha256": "...",
  "copy": "registry/model_registry.watchdog.yaml",
  "task_path": "agents/tasks/fmb_assembly_2.watchdog.yaml",
  "task_sha256": "...",
  "scenario_env": "S3",
  "override_env_seen": null,
  "limits_snapshot": {"u_brown": {"max_clamp_effort": 10.0}}
}
```

`registry_provenance_executor.json` takes the same fields, with `role: executor`, `override_env: "EXECUTOR_REGISTRY_PATH"` and `copy: "registry/model_registry.executor.yaml"`. It is written once at process start, not on each `get_rendered_task`.

`limits_snapshot` is a convenience copy. The registry copy is the authority. With the two copies in the session, a Class 4 run proves its own invariant: the WatchDog's `max_clamp_effort` is lower than the executor's.

---

## 5. Commands to run, in order

```bash
cd /home/mohammad/watchdog/WatchDog_V1

# 1. Fast pass, no bag reading (standard library only, any Python 3.10+)
python3 -m paper_media.audit.session_audit agents/transcripts/sessions \
    --out paper_media/audit/out/report_nobag.json --skip-bag

# 2. Full pass with bag checks (needs rosbags; the paper_media venv has it)
~/.venvs/paper_media/bin/python -m paper_media.audit.session_audit agents/transcripts/sessions \
    --out paper_media/audit/out/report.json

# 3. One session in detail
python3 -m paper_media.audit.session_audit agents/transcripts/sessions \
    --session 20261007T220959Z --out paper_media/audit/out/one.json --skip-bag

# 4. Read the output
python3 -c "import json; r=json.load(open('paper_media/audit/out/report.json')); print(json.dumps(r['summary'], indent=1))"
python3 -c "import json; r=json.load(open('paper_media/audit/out/report.json')); [print(s['session'], s['status'], [n for n,c in s['checks'].items() if c['status']!='ok']) for s in r['sessions'] if s['status']!='clean']"

# 5. Confirm the inferred causes
grep -rn '"rollup"' ros2_ws/src/robot_backend/test agents 2>/dev/null | head
grep -rln "EXECUTOR_TASK_PATH" ros2_ws/src/robot_backend/test 2>/dev/null | head
grep -rh "Session manifest" ~/.ros/log/ 2>/dev/null | sort | uniq -c | head
/usr/bin/python3 -c "import trimesh" && echo "trimesh present in system python"
```

Useful flags: `--fire-wake-timeout-s` (default 10), `--gap-s` (default 180), `--order-tol-s` (default 1), `--span-pad-s` (default 300).
