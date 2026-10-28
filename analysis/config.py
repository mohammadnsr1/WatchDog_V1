"""Campaign constants.  Everything a table builder must not infer from a run id."""
from __future__ import annotations

# The seven agent-condition batches.  Order is the row order used in tables.
CAMPAIGN_BATCHES: tuple[str, ...] = (
    "S1", "S2", "S3", "S4", "S3_opus_low", "S3_opus_medium", "S4_opus_medium",
)
# Observation-only oracle batch (scripted watchdog).  Never an agent condition.
ORACLE_BATCHES: tuple[str, ...] = ("REF",)
# Never loaded.
EXCLUDED_BATCHES: tuple[str, ...] = ("20261005_smoke",)

ALL_LOADED_BATCHES: tuple[str, ...] = CAMPAIGN_BATCHES + ORACLE_BATCHES

# Table 1 rows (Sonnet-low baseline), in order.
BASELINE_BATCHES: tuple[str, ...] = ("S1", "S2", "S3", "S4")
# Table 3 rows, in order.
CAPABILITY_BATCHES: tuple[str, ...] = (
    "S3", "S3_opus_low", "S3_opus_medium", "S4", "S4_opus_medium",
)

# Which role adjudicates in each scenario (the role whose tool call delivers a fire).
ADJUDICATING_ROLE: dict[str, str] = {"S1": "watchdog", "S2": "watchdog", "S3": "watchdog", "S4": "executor"}
GATED_SCENARIOS: frozenset[str] = frozenset({"S1", "S2", "S3"})

# Fraction-string columns in results[_s4].csv ("115/116").
FRACTION_COLUMNS: tuple[str, ...] = (
    "arming_coverage", "stationarity_coverage", "grip_coverage",
    "arming_provenance", "authored_param_accuracy",
)
# Parameter-accuracy source per scenario (pitfall 2).  Never summed across the two.
PARAM_ACCURACY_SOURCE: dict[str, str] = {
    "S1": "arming_provenance", "S2": "arming_provenance",
    "S3": "authored_param_accuracy", "S4": "authored_param_accuracy",
}

# S4-only columns passed through unchanged (no S1-S3 equivalent is synthesized).
S4_ONLY_COLUMNS: tuple[str, ...] = (
    "interleaved_monitor_blocks", "monitor_calls_in_run", "monitor_tool_wall_sec",
    "n_unmonitored_steps", "fire_to_adjudication_median_s", "fire_to_adjudication_p95_s",
    "fires_never_adjudicated", "executor_self_reported", "n_corrupted_binds",
    "class4_path_untested", "scored_registry_sha256", "run_registry_sha256",
    "registry_differs_from_run",
)
# Non-comparable latency columns, carried through under explicit names.
NONCOMPARABLE_LATENCY: dict[str, tuple[str, ...]] = {
    "S1-S3": ("fire_to_next_verdict_median_s", "fire_to_next_verdict_p95_s"),
    "S4": ("fire_to_adjudication_median_s", "fire_to_adjudication_p95_s"),
}

# MCP log hygiene: which servers belong to which role's log.
EXECUTOR_SERVERS: frozenset[str] = frozenset({"motion", "scene", "gripper", "executor_monitor"})
WATCHDOG_SERVERS: frozenset[str] = frozenset({"watchdog"})
# Motion/skill interval definition (D5): motion plus gripper servers.
MOTION_INTERVAL_SERVERS: frozenset[str] = frozenset({"motion", "gripper"})
# Tools whose return can carry a fire.
DELIVERY_TOOLS: frozenset[str] = frozenset({"wait_for_event", "get_monitor_fires"})

# Committed task: 12 steps + 4 grasped phases.
COMMITTED_TASK_GATES: int = 16

# Transcript mapping window around agent_config_<role>.json launched_wall_time_iso.
TRANSCRIPT_WINDOW_S: tuple[float, float] = (-60.0, 300.0)
TRANSCRIPT_ROLE_BY_SERVER: dict[str, str] = {
    "robot-backend-watchdog": "watchdog",
    "robot-backend-motion": "executor", "robot-backend-scene": "executor",
    "robot-backend-gripper": "executor", "robot-backend-executor-monitor": "executor",
}

# Voided sessions (A1): known_invalid entries, hand-classified into a named failure mode
# from the reason text recorded in batch.yaml.  The reason text itself is also emitted.
VOIDED_FAILURE_MODE: dict[str, str] = {
    "20261007T205057Z": "operator_error_leftover_intruder_prim_and_template_label",
    "20261026T011838Z": "watchdog_mcp_server_stall_on_first_wait_for_event_gate_timeout",
}

# Corrections classification (D8): which columns enter Tables 1-3.
TABLE_METRIC_COLUMNS: frozenset[str] = frozenset({
    "fault_TP", "fault_FP", "fault_FN", "fault_TN", "fault_recall", "fault_precision",
    "fault_TP_list", "fault_FP_list", "fault_FN_list", "fault_TP_by_fire", "fault_TP_by_check",
    "verdict_correct", "verdict_TP", "verdict_FP", "verdict_TN", "verdict_FN",
    "watchdog_final_verdict", "expected_final_verdict", "watchdog_recovered",
    "expected_recovered_ok", "n_real_violations", "arming_coverage", "grip_coverage",
    "stationarity_coverage", "arming_provenance", "authored_param_accuracy",
    "monitor_false_alarms", "independence_gap",
    "n_detected", "n_detectable", "obs_TP", "obs_FN", "obs_FP", "n_looks",
    "n_observation_monitors_armed", "n_observation_fires", "obs_monitor_false_fires",
    "detection_rate",
})
OBSERVATION_ADJUDICATION_COLUMNS: frozenset[str] = frozenset({
    "n_detected", "n_detectable", "n_undetectable", "n_episodes", "obs_TP", "obs_FN",
    "obs_FP", "obs_TN", "detection_rate", "detection_latency_median_s",
    "detection_latency_p95_s", "appearance_reported_accurate", "false_scene_reports",
    "n_unmatched_observations", "n_looks", "n_observation_monitors_armed",
    "n_observation_fires", "obs_monitor_false_fires", "n_record_observation_new",
    "n_authored_obs_monitors", "agent_chosen_obs_thresholds", "obs_flag_on_clause_reads",
})
