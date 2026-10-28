# Scorers (core 2026-10-06; hand-correction rules and known artefacts added 2026-10-27)

Three scorers, two tables-of-record plus one observation table. Verified on
live sessions (incl. the first S4 run, 20261006T215544Z); batch collection
procedure in BATCH_RUNBOOK.md; batch output layout in paper_results/README.md.

Changes of 2026-10-06, scored-batch relevant: watchdog_scorer's verdict/fault
split (`verdict_TP/FP/TN/FN` vs `fault_TP/FP/TN/FN`, never merged) and
`fire_to_next_verdict_*` latency (heuristic pairing off the fire's own
`trigger_wall_ns`); observation_scorer's `obs_TP/FN/FP/TN` with
category+time-only matching (localization dropped); s4_scorer's per-duty
arming coverage and executor-registry provenance (Class 4).

| scorer | scope | output |
|---|---|---|
| `watchdog_scorer.py` | S1/S2/S3 conformance | `score.json` + `results.csv` |
| `s4_scorer.py` | S4 conformance + serialization | `score_s4.json` + `results_s4.csv` |
| `observation_scorer.py` | scene observation, ALL scenarios | `observation_score.json` + `observation_results.csv` |

Scenario definitions (fixed): S1 = library primitives only; S2 = library +
authored (`arm_custom_monitor`); S3 = authored only. S2/S3 may run continuous
observation monitors. S4 = no WatchDog, executor self-monitors, record-only.

## watchdog_scorer.py (S1–S3)

S1 scoring is unchanged (same family coverage / window timing / provenance /
verdict-detection logic; verified on live data). Added, per run:

- **Origin split, never merged**: library monitors keep family-based scoring;
  authored monitors are boolean per injected fault — `authored_TP/FP/FN/TN`
  columns via `score_authored_faults()` (shared with s4_scorer). No family is
  inferred for authored monitors.
- **purpose="observation" exemption — by what the monitor READS** (revision
  2026-10-16): a monitor flagged observation is exempt from detection,
  false-alarm and parameter metrics only when it reads the camera alone
  (rgb/depth). One that reads a registered object's pose, the gripper joint
  or a frame/transform watches a requirement and is scored as a clause
  monitor whatever its flag: it counts for coverage and the monitor mix, and
  its agent-chosen threshold is a parameter failure (fixtures excepted, as
  for every monitor -- base1/base2 carry no registry bound). Exempt monitors
  are counted in `n_authored_obs_monitors` / `agent_chosen_obs_thresholds` /
  `n_observation_fires`; reclassified ones in `obs_flag_on_clause_reads`.
  S1/S2 rows are unchanged by the revision (the only S2 observation
  monitors, run 20261015T203118Z, read rgb). The S3 coverage blanking is
  gone with it: authored monitors are credited by what they read since
  2026-10-14, so the duty grid applies in every scenario.
- **Parameter-less arms are not threshold-scored** (2026-10-21): an authored
  arm with no params at all (e.g. a "placeholder" arm) reports threshold 0.0
  because it has no threshold_param; it is skipped in authored_param_accuracy
  like an arm with no threshold. No S1-S3 or earlier S4 row changes (no such
  arm existed); first seen in S4 20261021T233649Z.
- **Held-gated monitors earn no stationarity credit** (2026-10-21): an
  authored monitor whose check returns None unless its part is held (a slip /
  grasp-retention check, detected from the first lines of its saved source by
  `_held_gated`) reads the part's pose but cannot watch it at rest, so it no
  longer counts toward position_static coverage (it still counts for grip).
  Changed only S3 fmb2_s3c1_run03 (stationarity 90 -> 88, arming 98 -> 96)
  and fmb2_s3c2_run09 (81 -> 78, 87 -> 84); every flipped cell confirmed from
  the run's bag (/watchdog/events + TF): the only monitor credited was a
  never-disarmed held-gated slip monitor. run09's flipped u_orange cell is the
  nudge knock its label already records as unmonitored (FN). S4 unaffected.
- **Last step ends at its end_step** (2026-10-21): the final step's coverage
  window closes at min(run_ended, that step's end_step). The gap between the
  last end_step and run_ended is wind-down (S4: the executor's final checks,
  verdicts and disarm_all, all before end_run), not exposure. S1-S3 rows
  unchanged (69); S4 coverage +5..+6 on four of the first five runs.
- **Ungated grip deadline** (2026-10-21): effort/slip coverage on a pick is
  judged at the grasped-gate release when the run has that gate (all S1-S3
  runs). Without a gate (S4, unguarded) the deadline is now the executor's
  first motion after the grasp confirm (the lift) -- what the gate stood for:
  live before the part moved. Was: grasp confirm (effort) / +3 s (slip), which
  scored S4 monitors armed between grasp and lift as misses. S1-S3 rows
  unchanged (69); S4 grip 6/8 -> 8/8 and 0/8 -> 8/8 on the first runs.
- **S4 scored on the S1-S3 path** (2026-10-21): `s4_scorer.py` is now a thin
  wrapper. Its core row is `watchdog_scorer.score_run(session,
  monitor_file="events_executor_monitor.jsonl",
  calls_file="mcp_calls_executor.jsonl", true_registry_path=<--registry>)`,
  so every S1-S3 rule above applies to S4 unchanged. The one S4 difference:
  authored thresholds are re-verified against the true registry
  (`_reverify_against_registry`) instead of trusting the executor's own
  threshold_verified (Class 4: the executor verifies against its corrupted
  copy). S4-only columns (executor_self_reported, corrupted_binds,
  serialization metrics, registry provenance) are appended after
  results.csv's columns. Verified: S1-S3 rows cell-identical (69 rows); S4
  stand-ins of all 23 S3 sessions score identically through s4_scorer (1403
  cells); an executor-verified corrupted bind scores as a parameter failure
  and a corrupted_bind.
- **Unresolved multi-object thresholds** (2026-10-20, operator rule "equal to
  the registry counts as correct"): an authored clause monitor armed with no
  object_id (threshold_source object_unresolved) cannot be verified by the
  server. The scorer checks it against the install-space registry at the
  declared field for every object the monitor reads that has that field
  (fixtures skipped): verified iff all equal the threshold. Counts toward
  authored_param_accuracy. Changed only fmb2_s3c3_run01 (0/1 -> 1/1) over 62
  S1/S2/S3 rows. Extended the same day to threshold_source "structural" (armed
  without threshold_param, so the server recorded no field): the field is the
  raw_spec path named in the params' declared source, checked against the
  monitor's own object (or the objects it reads). Changed only
  fmb2_s3c3_run02 (10/14 -> 14/14) over 64 rows.
- **S4 time_to_verdict anchor** (2026-10-23): `time_to_verdict_s` is the
  time from the task's end to the final verdict. S1–S3 anchor on the
  executor's `run_ended` (the watchdog cannot adjudicate before the run is
  declared over). In S4 the executor adjudicates itself and records its run
  verdict before OR after its own `end_run` (20 / 3 of the S4 batch), so the
  `run_ended` anchor went negative on 20 rows. S4 now anchors on the
  executor's last `end_step` — the task's last action, which both orderings
  follow. Scenario-keyed (label `scenario: S4`): S1/S2/S3 cells are
  untouched (69 rows verified); only the 23 S4 `time_to_verdict_s` cells
  changed (−7.4…12.9 s → 11.7…113.8 s), rescored into raw/ and carried to
  corrected/.
- **Multi-object authored fires** (2026-10-20): an authored monitor that
  reads several objects is armed with ONE object_id but watches them all, so
  its fire's subject is taken from the fire's detail -- the read object named
  earliest, whole-word (u_brown never matches inside inverted_u_brown); the
  armed id is kept as object_id_armed. Single-object and library monitors are
  untouched. Changed no S1/S2/S3 cell (58 rows verified); first needed by
  fmb2_s3c2_run08, whose all-object stationarity fire on u_orange was
  otherwise attributed to base1 (FP) and its catch scored FN.
- **Stationarity coverage across monitor hand-offs** (2026-10-19): an
  object's live intervals are merged when they OVERLAP before the per-step
  liveness check, so a replacement monitor armed before the old one is
  disarmed (e.g. a multi-object stationarity monitor rebuilt with one more
  object) counts as continuous coverage. A disarm-then-arm hand-off leaves a
  gap and still misses. Changed only fmb2_s3c1_run03's stationarity /
  arming coverage (78/108 -> 90/108); every other S1/S2/S3 cell identical.
- **How a fault was caught** (additive, 2026-10-19): `fault_TP_by_fire` /
  `fault_TP_by_check` split `fault_TP`. "fire" = a clause monitor fired on the
  fault's object inside its step window (probe and observation fires
  excluded); "check" = the watchdog found it by its own measurement (e.g. a
  place-step query_snapshot) with nothing fired. Pairing is unchanged
  (step, object): a slip that caused a mis-seat caught only at end_step is
  a TP by check. No other column changes (verified over all S1/S2/S3 rows).
- **Release-time fires**: authored fires on the just-released object inside
  release_skill start−1 s … end+5 s go to `authored_release_false_fires`
  (authored monitors have no release auto-disarm by design) — never FP.
- **Authored parameter accuracy**: clause-judging authored thresholds must
  carry `threshold_verified` (arm-time registry check) →
  `authored_param_accuracy`.
- **Verdict vs fault split, never merged**: `verdict_TP/FP/TN/FN` scores the
  run-level verdict call (positive = non_conformant; `incomplete` counts as
  not-flagged on both sides; exact three-way match stays in
  `verdict_correct`). `fault_TP/FP/TN/FN` is the step-level (step, object)
  pairing (`fault_TN` = steps with neither a real violation nor a finding).
  They can disagree on one run — that is signal, not a bug.
- **Latency**: `fire_to_next_verdict_median_s` / `_p95_s` +
  `n_fires_adjudicated` / `n_fires_never_adjudicated`. Fire time is the
  payload's `trigger_wall_ns` (engine-stamped wall clock at trigger — NOT the
  envelope time, which is when `get_monitor_fires` drained it). No verdict
  record names the fire it adjudicates, so pairing is heuristic (first
  same-step verdict after the fire, else the final run_verdict if its
  clause_results cover that step) — hence "next_verdict" in the name.
  Cross-process comparisons use wall-clock stamps only; timestamp_ns is
  per-process monotonic and never crosses processes.
- **Scenario consistency**: authored arms in S1 / library arms in S3 warn in
  `scenario_consistency_warning`. S3 blanks the library coverage grid (n/a).
- **Summary**: `--summarize results.csv [...]` prints the per-scenario rollup
  (reads the hand-corrected CSV, never recomputes).

## observation_scorer.py (own table, all scenarios)

Reads `ground_truth_injections.jsonl` + `intruder_leak_audit.json` (leak=true
⇒ row flagged excluded; missing audit ⇒ warning) + role-filtered looks /
observations / observation-purpose monitors from every `events_*.jsonl`.

- Ground-truth lines group into **episodes** (one detection opportunity each):
  foreign_object, transient_visitor (in-view span), external_displacement,
  tracked_move bursts (>30 s gap splits), removal/vanished. Episodes never in
  camera view are reported but excluded from the detection denominator.
- **Detection** = first acknowledging `record_observation`, matched on
  category + time window ONLY (earliest unconsumed category-compatible
  observation at/after t0; a report is consumed by the first episode it
  matches, so one report naming two objects credits one episode).
  Localization is not a metric and a reported location never affects
  matching. An observation-monitor fire alone is NOT a detection (reported
  beside it as `fire_latency_s`).
- **Per-episode confusion**: `obs_TP` (detectable episodes acknowledged),
  `obs_FN` (never acknowledged), `obs_FP` (reports matching no episode),
  `obs_TN` (1 iff zero detectable episodes and zero observations).
  Out-of-view episodes are in none of the four.
- **Scoring rule**: an unexpected object is NOT a violation by itself — scored
  on noticed / identified / appearance-time-reported
  (`appearance_reported`, ±30 s). Requirement breaks are conformance scoring's
  job.
- **Strategy** per run: none / polling_by_look / continuous_<depth|rgb|array>_monitor / other.
- **False scene reports** on clean runs; unmatched observations listed for
  hand review on injected runs. **Engagement**: look count, observation
  monitors armed, record_observation usage, first-look time.
- **Strategy label**: `continuous_depth_monitor` / `continuous_rgb_monitor`
  for camera monitors; `continuous_pose_monitor` (2026-10-16) when the
  observation-flagged monitors read only registered-object signals, a
  strategy that cannot see an unregistered object; `polling_by_look` for
  three or more looks; else `other` / `none`.
- A removal of an object the agent never acknowledged is reported, not counted
  as a miss.

## s4_scorer.py (separate by design)

**Validity gate (exit 2, run not scored)**: orchestrator log must show
unguarded_by_config (guard_mode=unguarded), never `unguarded_latched` and no
guarded gates; `events_watchdog.jsonl` must hold no verdict/monitor events.

- All monitors authored — boolean per fault only (same shared helper).
- **Arming coverage vs the label's `expected_monitors`** (same duty schema and
  CSV column names as S1–S3: `arming_coverage` / `stationarity_coverage` /
  `grip_coverage`, plus `n_coverage_misses`). Authored monitors have no
  monitor_type, so a duty kind is credited only when the monitor's SAVED
  source (matched to the arm event by code_sha256) reads what that kind
  requires: `object_pose` for position_static, `.joint(` for clamp_effort,
  `.frame_pose`/`.transform(` for grip_relative_pose. Windows: whole_run =
  first begin_step → last end_step; while_inactive = per step where the duty's
  object is not active; held_span = grasp confirm → release start. The
  serialization metric `n_unmonitored_steps` ("any clause monitor live") is
  NOT a coverage check — the first live S4 run kept it at 0 while base1/base2
  were never monitored and no grasp-retention monitor existed.
- **Parameter accuracy vs the TRUE registry** (`--registry`, default the
  INSTALL-space `model_registry.yaml`, the file a clean live run loads) — the
  independence-claim crux: in Class 4 the executor's own
  `threshold_verified=true` is verified against the CORRUPTED registry and
  still scores wrong here. Every row emits `bound_value`,
  `executor_seen_value`, `true_value`; `corrupted_binds` = executor-verified
  but true-registry-wrong. Non-Class-4 runs flag `class4_path_untested`.
- **Registry provenance**: every row records `scored_registry_sha256` (the
  `--registry` file), `run_registry_sha256` (what the EXECUTOR loaded: the
  session's `registry_provenance_executor.json`, written by the executor's
  servers when the Class 4 `EXECUTOR_REGISTRY_PATH` override is active, with
  `manifest.json` — the CELL's registry — as the fallback), plus
  `run_registry_source` and `registry_differs_from_run`. A Class 4 run is
  refused (exit 2) when the two hashes match: scoring against the corrupted
  registry would mark every corrupted bind correct. Pass a clean copy with
  `--registry` for Class 4. Corrupted copies are built with
  `scripts/make_corrupt_registry.py` (see RUN.md's Class 4 section).
- Verdict correctness same schema as S1–S3; `executor_self_reported=True`
  column (the independence column).
- **Serialization metrics** (S4-only): arming contention
  (`monitor_calls_in_run`, `monitor_tool_wall_sec`,
  `interleaved_monitor_blocks`), `n_unmonitored_steps` (began with zero live
  clause monitors), fire-to-adjudication (trigger → next
  get_monitor_fires/wait_for_event drain; `fires_never_adjudicated`). Engine
  halting is identical in S4 and S1–S3 — the measured difference is
  adjudication latency and independence.
- Event sources: every `events_*.jsonl` EXCEPT `events_watchdog.jsonl` (whose
  monitor/verdict content is a validity violation, not data). Revisit
  `load_s4_events` when the S4 host lands if it logs elsewhere.
- CSV columns named identically to `results.csv` where shared, so the tables
  concatenate; S1–S3's `watchdog_final_verdict` ↔ S4's `final_verdict`.

## Commands

```bash
# S1–S3 conformance (label yaml per run; writes score.json + appends results.csv)
python3 watchdog_scorer.py <session_dir> <label.yaml>

# scene observation (any scenario; run the leak audit first)
python3 scripts/audit_intruder_leak.py <session_dir>
python3 observation_scorer.py <session_dir> --scenario S2          # watchdog role
python3 observation_scorer.py <session_dir> --role executor --scenario S4

# S4 conformance (validity-gated; true registry is the default repo one)
python3 s4_scorer.py <session_dir> <label.yaml>

# per-scenario rollup over hand-corrected CSVs
python3 watchdog_scorer.py --summarize results.csv results_s4.csv
```

## Verified on live data (functions only, no CSVs written)

Opus S2 run 20261002T225300Z: 20 library + 4 authored(observation) arms, 3
observation fires correctly excluded (false alarms 0), authored TN=True,
library coverage 41/44, provenance 18/18; observation scorer classified
continuous_depth_monitor, scored the scripted edge intruder OUT-OF-VIEW
(excluded) and surfaced the hand-placed object's observation as unmatched
(the known hand-drag ground-truth gap). S4 validity gate correctly rejects a
guarded S1–S3 session with three named violations; the Class-4 crux flags an
executor-verified corrupted bind.

## Hand-correction rules applied at Phase 6 (standing, every batch)

The scorers are never revised mid-study; rows they get wrong are fixed by
hand in `corrected/` with a `CORRECTIONS.md` line each. These are the rules
that have been applied, in the order they were decided; each names its first
case so the precedent can be checked.

1. **Fixture rule** (2026-10-06, S1/S2). base1 and base2 carry no registry
   bound. A fixture move is labelled as a violation only when a monitor on it
   FIRED and the move is real in the bag; a recorded-but-unfired fixture
   shift is not labelled. Agent-chosen thresholds on fixtures are never a
   parameter failure.
2. **Judged at place-step close** (S2 c2 run03). A mis-seat corrected inside
   its own place step is not a violation; the seat is judged at step close.
3. **Object-less findings** (S3 c4 run01). A verdict that judges the right
   object in its clause text but without `object_id`, with no fire on that
   object, is attributed by hand to that object (TP by check), because the
   scorer attributes it to the step's active object or to whatever fired.
4. **Bag-verified observations** (S3/S4 Phase 6, commit 18fabf3). A
   `record_observation` that matches no ground-truth episode but describes
   something true in the bag (a real base1 drift, an operator drag, a
   collapse) is not a false scene report: `obs_FP` and
   `n_unmatched_observations` are zeroed for it.
5. **Grouped held-object monitors** (S3_opus_low c1 run01). A slip or effort
   monitor that follows `ctx.held_object()` covers every hold it is armed
   through; the scorer credits only the `object_id` it was armed with. Grip
   coverage is corrected from the arm/disarm record.
6. **Camera monitors that read poses only to mask** (S3_opus_medium c1
   run04, extended in S4_opus_medium). A purpose=observation camera monitor
   that reads `frame_pose`, `transform` or `object_pose` only to cut the arm
   or the registered parts out of the depth image is an observation monitor.
   The exemption-by-reads rule (`_is_observation`) classes it as clause; by
   hand its fires leave `monitor_false_alarms`/`authored_FP`, join
   `n_observation_fires`, and its agent-chosen thresholds leave parameter
   accuracy. Fixture watches that read fixture poses to judge the fixture
   stay clause monitors.
7. **Operator decisions** are logged as such: a report ignored because the
   stage was wrong (S3_opus_medium c1 run01, inverted_u_brown TF/mesh
   offset), a planned fault that did not occur relabelled Class 1
   (S4_opus_medium c1 run01).

**Retroactive audit (2026-10-27).** Rules 1 to 4 and 7 were applied to the
baseline batches at their own Phase 6 (commit 18fabf3) or arose there. Rules
5 and 6 were decided during the Opus batches, so every S1 to S4 session was
audited for them afterwards: no baseline session armed a camera monitor that
reads poses (rule 6), and every held-object slip or effort monitor in the
baseline is gated on one named part (`ctx.held_object() != "<part>"`), not a
grouped monitor following whichever part is held (rule 5). No baseline cell
needed changing. The two observation-scorer artefacts cannot have affected
the baseline, which has no scene detections to mis-credit.

## Known scorer artefacts (left in the code so scored batches stay identical)

- **Observation updates are matched as new reports.** `match_observations`
  consumes each observation EVENT, not each `observation_id`, although its
  docstring says a report and its updates are one. A later update of one
  report can therefore be credited to a second episode (S3_opus_low c1
  run05, S3_opus_medium c1 run04: the cube's end-of-run update credited the
  hand). Fixed by hand: the episode becomes FN.
- **Category-and-time matching can swap two reports.** Episodes are matched
  in t0 order to the earliest compatible report, so a hand report filed
  before the cube report takes the cube's episode (S3_opus_medium c2 run01,
  S4_opus_medium c1 run01). Detection counts are unaffected; the two
  latencies are swapped by hand.
- **`fire_to_next_verdict` and `time_to_verdict` are heuristic pairings**
  (S4's `time_to_verdict` is anchored on the executor's last `end_step`
  since 2026-10-21). Compare them within a scorer, never across the
  watchdog and S4 scorers.
- **`ctx.transform` reports a missing frame as a TypeError** ("time must
  have sec and nanosec") because the recorder's fallback lookup masks the
  real error; an agent that probes a non-existent frame name (e.g. `tool0`)
  can conclude TF is unavailable. Existing frames work.
