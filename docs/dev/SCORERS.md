# Scorers (2026-10-02)

Three scorers, two tables-of-record plus one observation table. Nothing has
been run against pilot batches yet — commands below.

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
- **purpose="observation" exemption**: observation-purpose monitors (agent-
  chosen look triggers) are excluded from detection, false-alarm and parameter
  metrics; counted in `n_authored_obs_monitors` / `agent_chosen_obs_thresholds`
  / `n_observation_fires`.
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
  `--registry` file), `run_registry_sha256` (from the session's
  `manifest.json`, i.e. the registry the run loaded) and
  `registry_differs_from_run`. A Class 4 run is refused (exit 2) when the two
  hashes match: scoring against the corrupted registry would mark every
  corrupted bind correct. Pass a clean copy with `--registry` for Class 4.
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
