# Analysis audit

Generated 2026-10-28T19:52:33.113109+00:00

## Sessions attempted / scored / voided (A1)

**Attempted 110 / scored 108 / voided 2** (agent-condition batches; REF excluded; ('20261005_smoke',) never loaded).

Failure modes are hand-classified in analysis/config.py:VOIDED_FAILURE_MODE from the batch.yaml reason text, which is quoted here.

| batch | session | failure_mode | recorded reason |
|---|---|---|---|
| S1 | 20261007T205057Z | operator_error_leftover_intruder_prim_and_template_label | Operator error: a leftover cube (foreign_3124e9 at 0.45,-0.05) from the pre-run spot check sat in the scene all run with no place record; label was the fmb1 template; scored on pre-1d29e92 scorers. Its raw/ rows were removed in the 2026-10-07 cleanup (see notes). Not counted toward the 5 Class 1 runs. |
| S3_opus_medium | 20261026T011838Z | watchdog_mcp_server_stall_on_first_wait_for_event_gate_timeout | INFRASTRUCTURE: the watchdog MCP server stalled. The executor's begin_step for pick_u_brown (01:20:42) was published with 2 subscribers (bag /watchdog/events). The watchdog called wait_for_event at 01:21:12; the server never returned it -- not even at the call's own 180 s timeout (CLI log: still running at 210 s) -- and logged nothing after its three startup lines. The gate timed out at 01:22:42;  |

## Regression check vs provisional expectations (Phase 6)

Provisional hand-computed expectations (analysis/expectations.py) vs pipeline. A disagreement means one side has a bug; nothing is tuned to match.

| cell | column | expected | got | status |
|---|---|---|---|---|
| table1/S1 | arming_coverage | 2296/2478 | 2296/2478 | ok |
| table1/S1 | grip_coverage | 169/174 | 169/174 | ok |
| table1/S1 | verdict_correct | 22/23 | 22/23 | ok |
| table1/S1 | fault_recall | 33/36 | 33/36 | ok |
| table1/S1 | fault_precision | 33/37 | 33/37 | ok |
| table1/S1 | obs_detection | 0/16 | 0/16 | ok |
| table2/S1 | step_level_caught | 3/3 | 3/3 | ok |
| table1/S2 | arming_coverage | 2236/2414 | 2236/2414 | ok |
| table1/S2 | grip_coverage | 158/164 | 158/164 | ok |
| table1/S2 | verdict_correct | 19/23 | 19/23 | ok |
| table1/S2 | fault_recall | 33/33 | 33/33 | ok |
| table1/S2 | fault_precision | 33/33 | 33/33 | ok |
| table1/S2 | obs_detection | 0/18 | 0/18 | ok |
| table2/S2 | step_level_caught | 3/3 | 3/3 | ok |
| table1/S3 | arming_coverage | 2370/2626 | 2370/2626 | ok |
| table1/S3 | grip_coverage | 107/178 | 107/178 | ok |
| table1/S3 | verdict_correct | 22/23 | 22/23 | ok |
| table1/S3 | fault_recall | 25/33 | 25/33 | ok |
| table1/S3 | fault_precision | 25/26 | 25/26 | ok |
| table1/S3 | obs_detection | 0/22 | 0/22 | ok |
| table2/S3 | step_level_caught | 3/3 | 3/3 | ok |
| table1/S4 | arming_coverage | 1809/2726 | 1809/2726 | ok |
| table1/S4 | grip_coverage | 113/188 | 113/188 | ok |
| table1/S4 | verdict_correct | 18/23 | 18/23 | ok |
| table1/S4 | fault_recall | 29/49 | 29/49 | ok |
| table1/S4 | fault_precision | 29/36 | 29/36 | ok |
| table1/S4 | obs_detection | 0/22 | 0/22 | ok |
| table2/S4 | step_level_caught | 1/3 | 1/3 | ok |
| table3/S3 | n | 15 | 15 | ok |
| table3/S3 | arming_coverage | 1436/1611 | 1436/1611 | ok |
| table3/S3 | grip_coverage | 71/108 | 71/108 | ok |
| table3/S3 | verdict_correct | 15/15 | 15/15 | ok |
| table3/S3 | fault_recall | 14/21 | 14/21 | ok |
| table3/S3 | fault_precision | 14/15 | 14/15 | ok |
| table3/S3_opus_low | n | 6 | 6 | ok |
| table3/S3_opus_low | arming_coverage | 641/667 | 641/667 | ok |
| table3/S3_opus_low | grip_coverage | 46/46 | 46/46 | ok |
| table3/S3_opus_low | verdict_correct | 6/6 | 6/6 | ok |
| table3/S3_opus_low | fault_recall | 4/4 | 4/4 | ok |
| table3/S3_opus_low | fault_precision | 4/4 | 4/4 | ok |
| table3/S3_opus_medium | n | 5 | 5 | ok |
| table3/S3_opus_medium | arming_coverage | 613/616 | 613/616 | ok |
| table3/S3_opus_medium | grip_coverage | 40/40 | 40/40 | ok |
| table3/S3_opus_medium | verdict_correct | 5/5 | 5/5 | ok |
| table3/S3_opus_medium | fault_recall | 9/9 | 9/9 | ok |
| table3/S3_opus_medium | fault_precision | 9/9 | 9/9 | ok |
| table3/S4 | n | 15 | 15 | ok |
| table3/S4 | arming_coverage | 1191/1740 | 1191/1740 | ok |
| table3/S4 | grip_coverage | 80/120 | 80/120 | ok |
| table3/S4 | verdict_correct | 13/15 | 13/15 | ok |
| table3/S4 | fault_recall | 23/35 | 23/35 | ok |
| table3/S4 | fault_precision | 23/29 | 23/29 | ok |
| table3/S4_opus_medium | n | 5 | 5 | ok |
| table3/S4_opus_medium | arming_coverage | 503/504 | 503/504 | ok |
| table3/S4_opus_medium | grip_coverage | 36/36 | 36/36 | ok |
| table3/S4_opus_medium | verdict_correct | 5/5 | 5/5 | ok |
| table3/S4_opus_medium | fault_recall | 12/14 | 12/14 | ok |
| table3/S4_opus_medium | fault_precision | 12/14 | 12/14 | ok |

**0 disagreement(s).**

## Corrections density (D8)

Hand-adjustment density per batch (every logged edit resolved to the runs it touched; a batch-wide `*(all rows)*` edit counts once per run).

| batch | runs | corrections resolved to runs | per run | classification | column '-' | template rows skipped | malformed | by column |
|---|---|---|---|---|---|---|---|---|
| S1 | 23 | 25 | 1.09 | {'enters_tables(conformance)': 46, 'other(exclude)': 1, 'other(manual_note)': 1} | 0 | 0 | 0 | fault_TP_by_fire:23; fault_TP_by_check:23; exclude:1; manual_note:1 |
| S2 | 23 | 23 | 1.00 | {'enters_tables(conformance)': 46} | 0 | 1 | 0 | fault_TP_by_fire:23; fault_TP_by_check:23 |
| S3 | 23 | 42 | 1.83 | {'enters_tables(conformance)': 55, 'observation_adjudication': 6, 'other(manual_note)': 4} | 0 | 0 | 0 | fault_TP_by_check:24; fault_TP_by_fire:23; manual_note:4; obs_FP:3; n_unmatched_observations:3; fault_TP:1; fault_FP:1; fault_FN:1; fault_recall:1; fault_precision:1; fault_TP_list:1; fault_FP_list:1; fault_FN_list:1 |
| S4 | 23 | 62 | 2.70 | {'column_unspecified': 23, 'observation_adjudication': 20, 'other(manual_note)': 11, 'enters_tables(conformance)': 8} | 23 | 0 | 0 | manual_note:11; obs_FP:10; n_unmatched_observations:10; fault_TP:1; fault_FN:1; fault_recall:1; fault_precision:1; fault_TP_list:1; fault_FN_list:1; fault_TP_by_check:1; fault_FP:1 |
| S3_opus_low | 6 | 16 | 2.67 | {'enters_tables(conformance)': 10, 'observation_adjudication': 6} | 0 | 0 | 0 | grip_coverage:2; arming_coverage:2; obs_FP:1; fault_TP:1; fault_FN:1; fault_recall:1; fault_TP_list:1; fault_FN_list:1; fault_TP_by_check:1; n_detected:1; detection_rate:1; obs_TP:1; obs_FN:1; detection_latency_median_s:1 |
| S3_opus_medium | 5 | 28 | 5.60 | {'enters_tables(conformance)': 7, 'observation_adjudication': 15, 'other(authored_FP)': 1, 'other(authored_TN)': 1, 'other(fire_to_next_verdict_median_s)': 1, 'other(fire_to_next_verdict_p95_s)': 1, 'other(n_fires_adjudicated)': 1, 'other(n_fires_never_adjudicated)': 1} | 0 | 0 | 0 | obs_FP:3; n_unmatched_observations:3; detection_latency_p95_s:2; detection_latency_median_s:2; fault_TP:1; fault_FN:1; fault_recall:1; fault_TP_list:1; fault_FN_list:1; fault_TP_by_fire:1; n_detected:1; detection_rate:1; obs_TP:1; obs_FN:1; monitor_false_alarms:1; authored_FP:1; authored_TN:1; n_observation_fires:1; fire_to_next_verdict_median_s:1; fire_to_next_verdict_p95_s:1; n_fires_adjudicated:1; n_fires_never_adjudicated:1 |
| S4_opus_medium | 5 | 65 | 13.00 | {'enters_tables(conformance)': 21, 'other(authored_TP)': 1, 'other(authored_FP)': 4, 'observation_adjudication': 23, 'other(n_authored_clause_arms)': 4, 'other(fire_to_next_verdict_median_s)': 3, 'other(n_fires_adjudicated)': 3, 'other(authored_TN)': 1, 'other(fire_to_next_verdict_p95_s)': 2, 'other(n_fires_never_adjudicated)': 3} | 0 | 0 | 0 | obs_FP:5; n_unmatched_observations:5; monitor_false_alarms:4; authored_FP:4; authored_param_accuracy:4; n_observation_fires:4; n_authored_obs_monitors:4; agent_chosen_obs_thresholds:4; n_authored_clause_arms:4; fire_to_next_verdict_median_s:3; n_fires_adjudicated:3; n_fires_never_adjudicated:3; fault_TP:2; fault_TP_list:2; fire_to_next_verdict_p95_s:2; fault_FP:1; fault_precision:1; n_real_violations:1; fault_FP_list:1; fault_TP_by_fire:1; authored_TP:1; authored_TN:1; detection_latency_p95_s:1; fault_FN:1; fault_recall:1; fault_FN_list:1; fault_TP_by_check:1 |
| REF | 2 | 0 | 0.00 | {} | 0 | 0 | 0 |  |

Per-run counts:
| batch | run | n corrections | columns |
|---|---|---|---|
| S1 | fmb2_s1c1_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c1_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c1_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c1_run04 | 3 | exclude; fault_TP_by_check; fault_TP_by_fire; manual_note |
| S1 | fmb2_s1c1_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run06 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run07 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run08 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run09 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c3_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c3_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c2_run10 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c3_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c3_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c3_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c4_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c4_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S1 | fmb2_s1c4_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c1_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c1_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run06 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run07 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run08 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run09 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c2_run10 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c4_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c4_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c4_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c3_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c3_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c3_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c3_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c3_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c1_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c1_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S2 | fmb2_s2c1_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c1_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c1_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c1_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run04 | 4 | fault_TP_by_check; fault_TP_by_fire; manual_note; n_unmatched_observations; obs_FP |
| S3 | fmb2_s3c2_run05 | 4 | fault_TP_by_check; fault_TP_by_fire; manual_note; n_unmatched_observations; obs_FP |
| S3 | fmb2_s3c1_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run06 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run07 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run08 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run10 | 4 | fault_TP_by_check; fault_TP_by_fire; manual_note; n_unmatched_observations; obs_FP |
| S3 | fmb2_s3c3_run01 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c3_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c3_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c3_run04 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c3_run05 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c4_run01 | 11 | fault_FN; fault_FN_list; fault_FP; fault_FP_list; fault_TP; fault_TP_by_check; fault_TP_by_fire; fault_TP_list; fault_precision; fault_recall; manual_note |
| S3 | fmb2_s3c4_run02 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c4_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c1_run03 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S3 | fmb2_s3c2_run09 | 1 | fault_TP_by_check; fault_TP_by_fire |
| S4 | fmb2_s4c2_run01 | 1 |  |
| S4 | fmb2_s4c1_run01 | 1 |  |
| S4 | fmb2_s4c2_run02 | 1 |  |
| S4 | fmb2_s4c2_run03 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c2_run04 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c1_run02 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c1_run03 | 1 |  |
| S4 | fmb2_s4c1_run04 | 1 |  |
| S4 | fmb2_s4c2_run05 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c1_run05 | 1 |  |
| S4 | fmb2_s4c2_run06 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c2_run07 | 12 | fault_FN; fault_FN_list; fault_TP; fault_TP_by_check; fault_TP_list; fault_precision; fault_recall; manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c2_run08 | 1 |  |
| S4 | fmb2_s4c2_run09 | 2 | fault_FP |
| S4 | fmb2_s4c2_run10 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c3_run01 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c3_run02 | 1 |  |
| S4 | fmb2_s4c3_run03 | 1 |  |
| S4 | fmb2_s4c3_run04 | 1 |  |
| S4 | fmb2_s4c3_run05 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c4_run01 | 4 | manual_note; n_unmatched_observations; obs_FP |
| S4 | fmb2_s4c4_run02 | 1 |  |
| S4 | fmb2_s4c4_run03 | 1 |  |
| S3_opus_low | fmb2_s3ol_c1_run01 | 2 | arming_coverage; grip_coverage |
| S3_opus_low | fmb2_s3ol_c1_run02 | 1 | obs_FP |
| S3_opus_low | fmb2_s3ol_c2_run01 | 6 | fault_FN; fault_FN_list; fault_TP; fault_TP_by_check; fault_TP_list; fault_recall |
| S3_opus_low | fmb2_s3ol_c1_run05 | 7 | arming_coverage; detection_latency_median_s; detection_rate; grip_coverage; n_detected; obs_FN; obs_TP |
| S3_opus_medium | fmb2_s3om_c2_run01 | 10 | detection_latency_median_s; detection_latency_p95_s; fault_FN; fault_FN_list; fault_TP; fault_TP_by_fire; fault_TP_list; fault_recall; n_unmatched_observations; obs_FP |
| S3_opus_medium | fmb2_s3om_c1_run01 | 2 | n_unmatched_observations; obs_FP |
| S3_opus_medium | fmb2_s3om_c1_run03 | 2 | n_unmatched_observations; obs_FP |
| S3_opus_medium | fmb2_s3om_c1_run04 | 14 | authored_FP; authored_TN; detection_latency_median_s; detection_latency_p95_s; detection_rate; fire_to_next_verdict_median_s; fire_to_next_verdict_p95_s; monitor_false_alarms; n_detected; n_fires_adjudicated; n_fires_never_adjudicated; n_observation_fires; obs_FN; obs_TP |
| S4_opus_medium | fmb2_s4om_c2_run01 | 19 | agent_chosen_obs_thresholds; authored_FP; authored_TP; authored_param_accuracy; fault_FP; fault_FP_list; fault_TP; fault_TP_by_fire; fault_TP_list; fault_precision; fire_to_next_verdict_median_s; monitor_false_alarms; n_authored_clause_arms; n_authored_obs_monitors; n_fires_adjudicated; n_observation_fires; n_real_violations; n_unmatched_observations; obs_FP |
| S4_opus_medium | fmb2_s4om_c1_run01 | 15 | agent_chosen_obs_thresholds; authored_FP; authored_TN; authored_param_accuracy; detection_latency_p95_s; fire_to_next_verdict_median_s; fire_to_next_verdict_p95_s; monitor_false_alarms; n_authored_clause_arms; n_authored_obs_monitors; n_fires_adjudicated; n_fires_never_adjudicated; n_observation_fires; n_unmatched_observations; obs_FP |
| S4_opus_medium | fmb2_s4om_c2_run02 | 10 | agent_chosen_obs_thresholds; authored_FP; authored_param_accuracy; monitor_false_alarms; n_authored_clause_arms; n_authored_obs_monitors; n_fires_never_adjudicated; n_observation_fires; n_unmatched_observations; obs_FP |
| S4_opus_medium | fmb2_s4om_c2_run03 | 13 | agent_chosen_obs_thresholds; authored_FP; authored_param_accuracy; fire_to_next_verdict_median_s; fire_to_next_verdict_p95_s; monitor_false_alarms; n_authored_clause_arms; n_authored_obs_monitors; n_fires_adjudicated; n_fires_never_adjudicated; n_observation_fires; n_unmatched_observations; obs_FP |
| S4_opus_medium | fmb2_s4om_c2_run04 | 8 | fault_FN; fault_FN_list; fault_TP; fault_TP_by_check; fault_TP_list; fault_recall; n_unmatched_observations; obs_FP |
- orphan observation row skipped: {'batch_id': 'S3_opus_low', 'reason': 'blank_run_id_excluded_operator_row', 'manual_note': 'operator copy-paste error row - excluded', 'corrections': [{'date': '2026-10-26', 'file': 'observation_results.csv', 'columns': ['exclude'], 'old': '*(blank)*', 'new': '`1`', 'line': 18, 'column_unspecified': False}]}

## Gate timeouts vs arming coverage (D2 / A3)

Gate timeouts (120 s, executor proceeded) against score.json per_step_coverage at the timed-out step (A3). `missed` names the (object/monitor) pairs the scorer says were NOT armed in time at that step.

| batch | run | session | timed-out gate | arming_coverage | coverage at that step |
|---|---|---|---|---|---|
| S1 | fmb2_s1c2_run04 | 20261008T021346Z | place_approach_u_brown | 38/40 | place_approach_u_brown:expected=9,covered=8,missed=[u_orange/stationarity] |
| S3 | fmb2_s3c2_run04 | 20261019T230955Z | pick_u_orange | 72/116 | pick_u_orange:expected=11,covered=6,missed=[inverted_u_yellow/stationarity,line_green/stationarity,u_brown/stationarity,u_orange/effort,u_orange/slip] |
| S3 | fmb2_s3c2_run09 | 20261020T213417Z | pick_line_green | 84/87 | pick_line_green:expected=11,covered=10,missed=[u_orange/stationarity] |

## Adjusted vs scorer-raw fault counts (pitfall 5)

Runs where adjusted fault_* and scorer_raw_fault_* disagree, with the logged correction:

| batch | run | disagreement | corrections note |
|---|---|---|---|
| S3 | fmb2_s3c4_run01 | TP: adj=7 raw=6; FP: adj=0 raw=1; FN: adj=1 raw=2 | ('fault_TP_by_fire', 'fault_TP_by_check'): *(blank on 2 rows)*->recomputed per run: corrected/ initialised as a copy of raw/ (results.csv + observation_results.csv, Phase 6 step 1), then `scripts/backfill_tp_mode.py` filled the two additive TP- / ('fault_TP',): `6`->`7`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_FP',): `1`->`0`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_FN',): `2`->`1`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_recall',): `0.75`->`0.875`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_precision',): `0.8571428571428571`->`1.0`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_TP_list',): `pick_u_brown:u_brown;place_inverted_u_yellow:u_brown;place_line_green:line_green;recover_pick_line_green:base1;recover_pick_line_green:u_brown;recover_pick_line_green:u_orange`->`pick_u_brown:u_brown;place_inverted_u_yellow:line_green;place_inverted_u_yellow:u_brown;place_line_green:line_green;recover_pick_line_green:base1;recover_pick_line_green:u_brown;recover_pick_line_green:u_orange`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_FP_list',): `place_inverted_u_yellow:inverted_u_yellow`->`(blank)`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_FN_list',): `place_inverted_u_yellow:line_green;recover_place_line_green:line_green`->`recover_place_line_green:line_green`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra / ('fault_TP_by_check',): `0`->`1`: fmb2_s3c4_run01: the watchdog's second verdict at place_inverted_u_yellow (20:15:35, no object named) measured line_green at 0.02063 m against its placement fra |
| S4 | fmb2_s4c2_run07 | TP: adj=5 raw=4; FN: adj=3 raw=4 | ('fault_TP',): `4`->`5`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in / ('fault_FN',): `4`->`3`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in / ('fault_recall',): `0.5`->`0.625`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in / ('fault_precision',): `0.8`->`1.0`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in / ('fault_TP_list',): `pick_line_green:hex_red;place_line_green:line_green;recover_clear_line_green_obstruction:base1;recover_clear_line_green_obstruction:u_brown`->`pick_line_green:hex_red;place_inverted_u_yellow:inverted_u_yellow;place_line_green:line_green;recover_clear_line_green_obstruction:base1;recover_clear_line_green_obstruction:u_brown`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in / ('fault_FN_list',): `place_inverted_u_yellow:inverted_u_yellow;recover_clear_line_green_obstruction:inverted_u_yellow;recover_clear_line_green_obstruction:line_green;recover_clear_line_green_obstruction:u_orange`->`recover_clear_line_green_obstruction:inverted_u_yellow;recover_clear_line_green_obstruction:line_green;recover_clear_line_green_obstruction:u_orange`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in / ('fault_TP_by_check',): `1`->`2`: fmb2_s4c2_run07: the executor's non_conformant verdict at place_inverted_u_yellow (23:28:22, position_m 0.2 vs 0.012) IS the labelled place_inverted_u_yellow:in |
| S3_opus_low | fmb2_s3ol_c2_run01 | TP: adj=4 raw=3; FN: adj=0 raw=1 | ('fault_TP',): `3`->`4`: fmb2_s3ol_c2_run01: the watchdog's 23:19:03 verdict at place_line_green judged 'u_orange displacement from its seated rest pose 0.031 vs 0.02, pass false' -- th / ('fault_FN',): `1`->`0`: fmb2_s3ol_c2_run01: see fault_TP (u_orange reattributed FN -> TP by check). / ('fault_recall',): `0.75`->`1.0`: fmb2_s3ol_c2_run01: see fault_TP; recall = 4/4. Precision unchanged at 1.0. / ('fault_TP_list',): `place_line_green:base1;place_line_green:line_green;place_line_green:u_brown`->`place_line_green:base1;place_line_green:line_green;place_line_green:u_brown;place_line_green:u_orange`: fmb2_s3ol_c2_run01: see fault_TP. / ('fault_FN_list',): `place_line_green:u_orange`->*(blank)*: fmb2_s3ol_c2_run01: see fault_TP. / ('fault_TP_by_check',): `0`->`1`: fmb2_s3ol_c2_run01: the reattributed u_orange TP came from the watchdog's own measurement, not a fire. authored_TP/FN are left as scored: no u_orange monitor fi |
| S3_opus_medium | fmb2_s3om_c2_run01 | TP: adj=9 raw=8; FN: adj=0 raw=1 | ('fault_TP',): `8`->`9`: fmb2_s3om_c2_run01: place_inverted_u_yellow:base1 was caught. The agent armed its base1 re-arm monitor (still_base1_rearm) with object_id u_orange, so its fire  / ('fault_FN',): `1`->`0`: fmb2_s3om_c2_run01: see fault_TP. / ('fault_recall',): `0.8888888888888888`->`1.0`: fmb2_s3om_c2_run01: see fault_TP; recall 9/9. Precision unchanged at 1.0. / ('fault_TP_list',): `place_approach_u_orange:u_orange;place_inverted_u_yellow:inverted_u_yellow;place_inverted_u_yellow:line_green;place_inverted_u_yellow:u_brown;place_inverted_u_yellow:u_orange;place_u_brown:u_brown;place_u_orange:u_orange;reseat_pick_u_orange:base1`->`place_approach_u_orange:u_orange;place_inverted_u_yellow:base1;place_inverted_u_yellow:inverted_u_yellow;place_inverted_u_yellow:line_green;place_inverted_u_yellow:u_brown;place_inverted_u_yellow:u_orange;place_u_brown:u_brown;place_u_orange:u_orange;reseat_pick_u_orange:base1`: fmb2_s3om_c2_run01: see fault_TP. / ('fault_FN_list',): `place_inverted_u_yellow:base1`->*(blank)*: fmb2_s3om_c2_run01: see fault_TP. / ('fault_TP_by_fire',): `5`->`6`: fmb2_s3om_c2_run01: the reattributed base1 TP came from a monitor fire on base1. |
| S4_opus_medium | fmb2_s4om_c2_run01 | TP: adj=2 raw=1; FP: adj=2 raw=3 | ('fault_TP',): `1`->`2`: fmb2_s4om_c2_run01: base1 at place_u_orange is a labelled violation by the fixture rule (bag: base1 8.8 mm peak at 22:26:24; obs_base_fixtures fired 0.0086); th / ('fault_FP',): `3`->`2`: fmb2_s4om_c2_run01: see fault_TP; the remaining FPs are place_u_orange:u_brown (bag 15.3 mm < 0.02, monitor armed at position_m 0.01) and place_inverted_u_yello / ('fault_precision',): `0.25`->`0.5`: fmb2_s4om_c2_run01: 2 TP / 4 findings. / ('fault_TP_list',): `place_line_green:line_green`->`place_line_green:line_green;place_u_orange:base1`: fmb2_s4om_c2_run01: see fault_TP. / ('fault_FP_list',): `place_inverted_u_yellow:inverted_u_yellow;place_u_orange:base1;place_u_orange:u_brown`->`place_inverted_u_yellow:inverted_u_yellow;place_u_orange:u_brown`: fmb2_s4om_c2_run01: see fault_TP. / ('fault_TP_by_fire',): `1`->`2`: fmb2_s4om_c2_run01: base1 caught by obs_base_fixtures' fire. |
| S4_opus_medium | fmb2_s4om_c2_run04 | TP: adj=7 raw=6; FN: adj=0 raw=1 | ('fault_TP',): `6`->`7`: fmb2_s4om_c2_run04: place_inverted_u_yellow:inverted_u_yellow was caught. The executor's 20:17:59 verdict judged 'inverted_u_yellow position error to placement  / ('fault_FN',): `1`->`0`: fmb2_s4om_c2_run04: see fault_TP. / ('fault_recall',): `0.8571428571428571`->`1.0`: fmb2_s4om_c2_run04: 7/7. / ('fault_TP_list',): `place_approach_u_orange:hex_red;place_inverted_u_yellow:base1;place_inverted_u_yellow:line_green;place_inverted_u_yellow:u_brown;place_inverted_u_yellow:u_orange;place_line_green:base1`->`place_approach_u_orange:hex_red;place_inverted_u_yellow:base1;place_inverted_u_yellow:inverted_u_yellow;place_inverted_u_yellow:line_green;place_inverted_u_yellow:u_brown;place_inverted_u_yellow:u_orange;place_line_green:base1`: fmb2_s4om_c2_run04: see fault_TP. / ('fault_FN_list',): `place_inverted_u_yellow:inverted_u_yellow`->*(blank)*: fmb2_s4om_c2_run04: see fault_TP. / ('fault_TP_by_check',): `0`->`1`: fmb2_s4om_c2_run04: the reattributed TP came from the executor's own measurement; authored_FN left as scored (no monitor fired on inverted_u_yellow). |

## Transcript mapping (2e)

Transcript dir: `/home/mohammad/.claude/projects/-home-mohammad-watchdog-WatchDog-V1` (351 transcripts indexed).
Rule: role from MCP server names; session = the one transcript whose first record is within [-60, +300] s of agent_config_<role>.json launched_wall_time_iso.

| role | status | n |
|---|---|---|
| executor | unique | 108 |
| watchdog | no_watchdog_role | 28 |
| watchdog | unique | 80 |

Unmapped role-sessions: 0

## Log hygiene, clock domains, non-comparable columns

MCP call logs are filtered to the role's own servers AND, per server, that server's modal pid. Records dropped:

| batch | run | executor records dropped | foreign servers | watchdog records dropped |
|---|---|---|---|---|
| S1 | fmb2_s1c1_run03 | 71 | watchdog:71 | 0 |
| S1 | fmb2_s1c2_run01 | 71 | watchdog:71 | 0 |
| S1 | fmb2_s1c1_run04 | 71 | watchdog:71 | 0 |
| S1 | fmb2_s1c1_run05 | 71 | watchdog:71 | 0 |
| S1 | fmb2_s1c2_run05 | 72 | watchdog:71 | 0 |
| S1 | fmb2_s1c2_run07 | 71 | watchdog:71 | 0 |
| S1 | fmb2_s1c2_run09 | 142 | watchdog:142 | 0 |
| S1 | fmb2_s1c3_run05 | 71 | watchdog:71 | 0 |
| S1 | fmb2_s1c4_run03 | 213 | watchdog:213 | 0 |
| S2 | fmb2_s2c2_run01 | 71 | watchdog:71 | 0 |
| S2 | fmb2_s2c2_run02 | 142 | watchdog:142 | 0 |
| S2 | fmb2_s2c2_run07 | 142 | watchdog:142 | 0 |
| S2 | fmb2_s2c1_run05 | 1 |  | 0 |

Clock domains: orchestrator wall_ns (epoch), waited_sec (monotonic delta, same process); MCP wall_time_iso = call start, return = start + elapsed_sec; event envelopes wall_time_iso; fire payload trigger_wall_ns (epoch ns). All on one host. The monotonic envelope timestamp_ns is never used. Gate request time = grant wall_ns - waited_sec.

Delivery asymmetry (D4): S1-S3 delivery is the return of a BLOCKING wait_for_event; S4 delivery is the return of a get_monitor_fires POLL, so S4 latency is set by when the executor next chose to poll. `delivery_mechanism` carries this per row.

Non-comparable columns carried through unchanged: fire_to_next_verdict_* (S1-S3) and fire_to_adjudication_* (S4) are different quantities; interleaved_monitor_blocks / monitor_calls_in_run / monitor_tool_wall_sec are S4-only by construction.

watchdog_postrun_tail_s (run_ended -> last watchdog MCP return) overlaps time_to_verdict_s (= watchdog_adjudication_s); they describe the same post-run interval from two anchors and must never be summed.

## Every per-run NA

| column | reason | n runs | runs |
|---|---|---|---|
| cache_creation_tokens_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| cache_read_tokens_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| fire_delivery_max_s | no_fires_in_run | 28 | S1/fmb2_s1c1_run01, S1/fmb2_s1c1_run02, S1/fmb2_s1c1_run03, S1/fmb2_s1c1_run04, S1/fmb2_s1c1_run05, S1/fmb2_s1c3_run01, S1/fmb2_s1c3_run03, S2/fmb2_s2c2_run01 (+20) |
| fire_delivery_median_s | no_fires_in_run | 28 | S1/fmb2_s1c1_run01, S1/fmb2_s1c1_run02, S1/fmb2_s1c1_run03, S1/fmb2_s1c1_run04, S1/fmb2_s1c1_run05, S1/fmb2_s1c3_run01, S1/fmb2_s1c3_run03, S2/fmb2_s2c2_run01 (+20) |
| fire_delivery_p95_s | no_fires_in_run | 28 | S1/fmb2_s1c1_run01, S1/fmb2_s1c1_run02, S1/fmb2_s1c1_run03, S1/fmb2_s1c1_run04, S1/fmb2_s1c1_run05, S1/fmb2_s1c3_run01, S1/fmb2_s1c3_run03, S2/fmb2_s2c2_run01 (+20) |
| frac_fires_during_gripper_server | no_fires_in_run | 28 | S1/fmb2_s1c1_run01, S1/fmb2_s1c1_run02, S1/fmb2_s1c1_run03, S1/fmb2_s1c1_run04, S1/fmb2_s1c1_run05, S1/fmb2_s1c3_run01, S1/fmb2_s1c3_run03, S2/fmb2_s2c2_run01 (+20) |
| frac_fires_during_motion | no_fires_in_run | 28 | S1/fmb2_s1c1_run01, S1/fmb2_s1c1_run02, S1/fmb2_s1c1_run03, S1/fmb2_s1c1_run04, S1/fmb2_s1c1_run05, S1/fmb2_s1c3_run01, S1/fmb2_s1c3_run03, S2/fmb2_s2c2_run01 (+20) |
| frac_fires_during_motion_server | no_fires_in_run | 28 | S1/fmb2_s1c1_run01, S1/fmb2_s1c1_run02, S1/fmb2_s1c1_run03, S1/fmb2_s1c1_run04, S1/fmb2_s1c1_run05, S1/fmb2_s1c3_run01, S1/fmb2_s1c3_run03, S2/fmb2_s2c2_run01 (+20) |
| gate_overhead_fraction | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| gate_overhead_fraction_ex_timeout | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| gate_wait_max_s | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| gate_wait_median_s | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| gate_wait_p95_s | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| gate_wait_total_ex_timeout_s | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| gate_wait_total_s | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| n_assistant_msgs_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| n_gate_timeouts | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| n_gates | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| tokens_in_executor | oracle_batch_not_an_agent_condition | 2 | REF/20261025T200836Z, REF/20261025T202724Z |
| tokens_in_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| tokens_in_watchdog | oracle_batch_not_an_agent_condition | 2 | REF/20261025T200836Z, REF/20261025T202724Z |
| tokens_out_executor | oracle_batch_not_an_agent_condition | 2 | REF/20261025T200836Z, REF/20261025T202724Z |
| tokens_out_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| tokens_out_watchdog | oracle_batch_not_an_agent_condition | 2 | REF/20261025T200836Z, REF/20261025T202724Z |
| transcript_model_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| transcript_watchdog | no_watchdog_role | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| watchdog_adjudication_s | no_watchdog_role_s4_time_to_verdict_not_comparable | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |
| watchdog_adjudication_s | time_to_verdict_s_blank | 2 | REF/20261025T200836Z, REF/20261025T202724Z |
| watchdog_postrun_tail_s | no_gates_in_scenario | 28 | S4/fmb2_s4c2_run01, S4/fmb2_s4c1_run01, S4/fmb2_s4c2_run02, S4/fmb2_s4c2_run03, S4/fmb2_s4c2_run04, S4/fmb2_s4c1_run02, S4/fmb2_s4c1_run03, S4/fmb2_s4c1_run04 (+20) |

## table1_baseline

| row | n included | n excluded | excluded (run: reason) |
|---|---|---|---|
| S1 | 23 | 1 | fmb2_s1c1_run04: observation_row_excluded:exclude_flag |
| S2 | 23 | 0 | - |
| S3 | 23 | 0 | - |
| S4 | 23 | 0 | - |

NA cells:
| row | column | reason |
|---|---|---|
| S1 | obs_false_fire_fraction | no_observation_fires |
| S4 | gate_overhead_fraction | no_gates_in_scenario |
| S4 | gate_overhead_fraction_ex_timeout | no_gates_in_scenario |
| S4 | gate_wait_median_s | no_gates_in_scenario |

Numerator / denominator behind every pooled cell:
| row | column | derivation |
|---|---|---|
| S1 | arming_coverage | 2296/2478 from arming_coverage over 23 runs |
| S1 | grip_coverage | 169/174 from grip_coverage over 23 runs |
| S1 | param_accuracy | 333/383 from arming_provenance over 23 runs |
| S1 | verdict_correct | 22/23 = runs with verdict_correct==True / included runs |
| S1 | fault_recall | 33/36 = sum fault_TP / (sum TP + sum FN) |
| S1 | fault_precision | 33/37 = sum fault_TP / (sum TP + sum FP) |
| S1 | scorer_raw_fault_recall | 33/36 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S1 | scorer_raw_fault_precision | 33/37 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S1 | obs_engagement | 15/19 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S1 | obs_detection | 0/16 = sum obs n_detected / sum obs n_detectable |
| S1 | obs_false_fire_fraction | 0/0 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S1 | false_alarm_rate | 0/9 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S1 | gate_overhead_fraction | sum gate_wait_total_s 3737.6 / sum run_wall_s 12708.8 over 23 runs |
| S1 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 3617.6 / sum run_wall_s 12708.8 |
| S1 | gate_wait_median_s | median over 379 non-timeout gate waits pooled across runs |
| S1 | independence_gap | 6/23 = runs with independence_gap==True / included runs |
| S2 | arming_coverage | 2236/2414 from arming_coverage over 23 runs |
| S2 | grip_coverage | 158/164 from grip_coverage over 23 runs |
| S2 | param_accuracy | 334/350 from arming_provenance over 23 runs |
| S2 | verdict_correct | 19/23 = runs with verdict_correct==True / included runs |
| S2 | fault_recall | 33/33 = sum fault_TP / (sum TP + sum FN) |
| S2 | fault_precision | 33/33 = sum fault_TP / (sum TP + sum FP) |
| S2 | scorer_raw_fault_recall | 33/33 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S2 | scorer_raw_fault_precision | 33/33 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S2 | obs_engagement | 10/18 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S2 | obs_detection | 0/18 = sum obs n_detected / sum obs n_detectable |
| S2 | obs_false_fire_fraction | 3/3 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S2 | false_alarm_rate | 0/9 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S2 | gate_overhead_fraction | sum gate_wait_total_s 3141.2 / sum run_wall_s 13206.4 over 23 runs |
| S2 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 3141.2 / sum run_wall_s 13206.4 |
| S2 | gate_wait_median_s | median over 364 non-timeout gate waits pooled across runs |
| S2 | independence_gap | 1/23 = runs with independence_gap==True / included runs |
| S3 | arming_coverage | 2370/2626 from arming_coverage over 23 runs |
| S3 | grip_coverage | 107/178 from grip_coverage over 23 runs |
| S3 | param_accuracy | 334/339 from authored_param_accuracy over 22 runs; blank on ['fmb2_s3c2_run08'] |
| S3 | verdict_correct | 22/23 = runs with verdict_correct==True / included runs |
| S3 | fault_recall | 25/33 = sum fault_TP / (sum TP + sum FN) |
| S3 | fault_precision | 25/26 = sum fault_TP / (sum TP + sum FP) |
| S3 | scorer_raw_fault_recall | 24/33 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S3 | scorer_raw_fault_precision | 24/26 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S3 | obs_engagement | 21/22 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3 | obs_detection | 0/22 = sum obs n_detected / sum obs n_detectable |
| S3 | obs_false_fire_fraction | 142/144 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3 | false_alarm_rate | 0/13 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S3 | gate_overhead_fraction | sum gate_wait_total_s 5084.7 / sum run_wall_s 14996.8 over 23 runs |
| S3 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 4844.7 / sum run_wall_s 14996.8 |
| S3 | gate_wait_median_s | median over 390 non-timeout gate waits pooled across runs |
| S3 | independence_gap | 2/23 = runs with independence_gap==True / included runs |
| S4 | arming_coverage | 1809/2726 from arming_coverage over 23 runs |
| S4 | grip_coverage | 113/188 from grip_coverage over 23 runs |
| S4 | param_accuracy | 233/240 from authored_param_accuracy over 23 runs |
| S4 | verdict_correct | 18/23 = runs with verdict_correct==True / included runs |
| S4 | fault_recall | 29/49 = sum fault_TP / (sum TP + sum FN) |
| S4 | fault_precision | 29/36 = sum fault_TP / (sum TP + sum FP) |
| S4 | scorer_raw_fault_recall | 28/49 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S4 | scorer_raw_fault_precision | 28/35 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S4 | obs_engagement | 21/22 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4 | obs_detection | 0/22 = sum obs n_detected / sum obs n_detectable |
| S4 | obs_false_fire_fraction | 10/10 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4 | false_alarm_rate | 2/10 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S4 | independence_gap | 7/23 = runs with independence_gap==True / included runs |


## table2_independence

| row | n included | n excluded | excluded (run: reason) |
|---|---|---|---|
| S1 | 3 | 0 | - |
| S2 | 3 | 0 | - |
| S3 | 3 | 0 | - |
| S4 | 3 | 0 | - |

Numerator / denominator behind every pooled cell:
| row | column | derivation |
|---|---|---|
| S1 | verdict_correct | 2/3 = runs with verdict_correct==True / included runs |
| S1 | step_level_caught | 3/3 = runs with fault_TP>=1 / class-4 runs |
| S2 | verdict_correct | 0/3 = runs with verdict_correct==True / included runs |
| S2 | step_level_caught | 3/3 = runs with fault_TP>=1 / class-4 runs |
| S3 | verdict_correct | 2/3 = runs with verdict_correct==True / included runs |
| S3 | step_level_caught | 3/3 = runs with fault_TP>=1 / class-4 runs |
| S4 | verdict_correct | 1/3 = runs with verdict_correct==True / included runs |
| S4 | step_level_caught | 1/3 = runs with fault_TP>=1 / class-4 runs |

- Fisher exact one-sided (S1-S3 pooled caught 9/9 > S4 caught 1/3): p = 0.0455

## table3_capability

| row | n included | n excluded | excluded (run: reason) |
|---|---|---|---|
| S3 | 15 | 0 | - |
| S3_opus_low | 6 | 0 | - |
| S3_opus_medium | 5 | 0 | - |
| S4 | 15 | 0 | - |
| S4_opus_medium | 5 | 0 | - |

NA cells:
| row | column | reason |
|---|---|---|
| S3 | S4_fire_to_adjudication_median_s_median | not_S4 |
| S3_opus_low | S4_fire_to_adjudication_median_s_median | not_S4 |
| S3_opus_medium | S4_fire_to_adjudication_median_s_median | not_S4 |
| S4 | S1S3_fire_to_next_verdict_median_s_median | not_S1S3 |
| S4 | cache_creation_tokens_watchdog_median_per_run | no_watchdog_role |
| S4 | cache_creation_tokens_watchdog_sum | no_watchdog_role |
| S4 | cache_read_tokens_watchdog_median_per_run | no_watchdog_role |
| S4 | cache_read_tokens_watchdog_sum | no_watchdog_role |
| S4 | gate_overhead_fraction | no_gates_in_scenario |
| S4 | gate_overhead_fraction_ex_timeout | no_gates_in_scenario |
| S4 | gate_wait_median_s | no_gates_in_scenario |
| S4 | tokens_in_watchdog_median_per_run | no_watchdog_role |
| S4 | tokens_in_watchdog_sum | no_watchdog_role |
| S4 | tokens_out_watchdog_median_per_run | no_watchdog_role |
| S4 | tokens_out_watchdog_sum | no_watchdog_role |
| S4_opus_medium | S1S3_fire_to_next_verdict_median_s_median | not_S1S3 |
| S4_opus_medium | cache_creation_tokens_watchdog_median_per_run | no_watchdog_role |
| S4_opus_medium | cache_creation_tokens_watchdog_sum | no_watchdog_role |
| S4_opus_medium | cache_read_tokens_watchdog_median_per_run | no_watchdog_role |
| S4_opus_medium | cache_read_tokens_watchdog_sum | no_watchdog_role |
| S4_opus_medium | gate_overhead_fraction | no_gates_in_scenario |
| S4_opus_medium | gate_overhead_fraction_ex_timeout | no_gates_in_scenario |
| S4_opus_medium | gate_wait_median_s | no_gates_in_scenario |
| S4_opus_medium | tokens_in_watchdog_median_per_run | no_watchdog_role |
| S4_opus_medium | tokens_in_watchdog_sum | no_watchdog_role |
| S4_opus_medium | tokens_out_watchdog_median_per_run | no_watchdog_role |
| S4_opus_medium | tokens_out_watchdog_sum | no_watchdog_role |

Numerator / denominator behind every pooled cell:
| row | column | derivation |
|---|---|---|
| S3 | arming_coverage | 1436/1611 from arming_coverage over 15 runs |
| S3 | grip_coverage | 71/108 from grip_coverage over 15 runs |
| S3 | param_accuracy | 218/223 from authored_param_accuracy over 14 runs; blank on ['fmb2_s3c2_run08'] |
| S3 | verdict_correct | 15/15 = runs with verdict_correct==True / included runs |
| S3 | fault_recall | 14/21 = sum fault_TP / (sum TP + sum FN) |
| S3 | fault_precision | 14/15 = sum fault_TP / (sum TP + sum FP) |
| S3 | scorer_raw_fault_recall | 14/21 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S3 | scorer_raw_fault_precision | 14/15 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S3 | obs_engagement | 14/14 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3 | obs_detection | 0/14 = sum obs n_detected / sum obs n_detectable |
| S3 | obs_false_fire_fraction | 71/71 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3 | false_alarm_rate | 0/10 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S3 | gate_overhead_fraction | sum gate_wait_total_s 3473.8 / sum run_wall_s 9291.8 over 15 runs |
| S3 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 3233.8 / sum run_wall_s 9291.8 |
| S3 | gate_wait_median_s | median over 238 non-timeout gate waits pooled across runs |
| S3 | independence_gap | 0/15 = runs with independence_gap==True / included runs |
| S3 | obs_engagement_wholebatch | 21/22 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3 | obs_detection_wholebatch | 0/22 = sum obs n_detected / sum obs n_detectable |
| S3 | obs_false_fire_fraction_wholebatch | 142/144 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3 | frac_fires_during_motion | 84/92 fires inside a motion-or-gripper call interval (pooled over runs) |
| S3 | fire_delivery_median_s | median over 92 delivered fires pooled across runs |
| S3 | tokens_executor | summed over 15/15 runs with a uniquely mapped executor transcript |
| S3 | tokens_watchdog | summed over 15/15 runs with a uniquely mapped watchdog transcript |
| S3_opus_low | arming_coverage | 641/667 from arming_coverage over 6 runs |
| S3_opus_low | grip_coverage | 46/46 from grip_coverage over 6 runs |
| S3_opus_low | param_accuracy | 64/64 from authored_param_accuracy over 6 runs |
| S3_opus_low | verdict_correct | 6/6 = runs with verdict_correct==True / included runs |
| S3_opus_low | fault_recall | 4/4 = sum fault_TP / (sum TP + sum FN) |
| S3_opus_low | fault_precision | 4/4 = sum fault_TP / (sum TP + sum FP) |
| S3_opus_low | scorer_raw_fault_recall | 3/4 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S3_opus_low | scorer_raw_fault_precision | 3/3 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S3_opus_low | obs_engagement | 6/6 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3_opus_low | obs_detection | 5/9 = sum obs n_detected / sum obs n_detectable |
| S3_opus_low | obs_false_fire_fraction | 2/2 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3_opus_low | false_alarm_rate | 0/5 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S3_opus_low | gate_overhead_fraction | sum gate_wait_total_s 844.6 / sum run_wall_s 3224.1 over 6 runs |
| S3_opus_low | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 844.6 / sum run_wall_s 3224.1 |
| S3_opus_low | gate_wait_median_s | median over 97 non-timeout gate waits pooled across runs |
| S3_opus_low | independence_gap | 0/6 = runs with independence_gap==True / included runs |
| S3_opus_low | obs_engagement_wholebatch | 6/6 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3_opus_low | obs_detection_wholebatch | 5/9 = sum obs n_detected / sum obs n_detectable |
| S3_opus_low | obs_false_fire_fraction_wholebatch | 2/2 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3_opus_low | frac_fires_during_motion | 5/5 fires inside a motion-or-gripper call interval (pooled over runs) |
| S3_opus_low | fire_delivery_median_s | median over 5 delivered fires pooled across runs |
| S3_opus_low | tokens_executor | summed over 6/6 runs with a uniquely mapped executor transcript |
| S3_opus_low | tokens_watchdog | summed over 6/6 runs with a uniquely mapped watchdog transcript |
| S3_opus_medium | arming_coverage | 613/616 from arming_coverage over 5 runs |
| S3_opus_medium | grip_coverage | 40/40 from grip_coverage over 5 runs |
| S3_opus_medium | param_accuracy | 86/86 from authored_param_accuracy over 5 runs |
| S3_opus_medium | verdict_correct | 5/5 = runs with verdict_correct==True / included runs |
| S3_opus_medium | fault_recall | 9/9 = sum fault_TP / (sum TP + sum FN) |
| S3_opus_medium | fault_precision | 9/9 = sum fault_TP / (sum TP + sum FP) |
| S3_opus_medium | scorer_raw_fault_recall | 8/9 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S3_opus_medium | scorer_raw_fault_precision | 8/8 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S3_opus_medium | obs_engagement | 5/5 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3_opus_medium | obs_detection | 9/10 = sum obs n_detected / sum obs n_detectable |
| S3_opus_medium | obs_false_fire_fraction | 5/10 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3_opus_medium | false_alarm_rate | 0/4 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S3_opus_medium | gate_overhead_fraction | sum gate_wait_total_s 1728.1 / sum run_wall_s 3699.7 over 5 runs |
| S3_opus_medium | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 1728.1 / sum run_wall_s 3699.7 |
| S3_opus_medium | gate_wait_median_s | median over 92 non-timeout gate waits pooled across runs |
| S3_opus_medium | independence_gap | 0/5 = runs with independence_gap==True / included runs |
| S3_opus_medium | obs_engagement_wholebatch | 5/5 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3_opus_medium | obs_detection_wholebatch | 9/10 = sum obs n_detected / sum obs n_detectable |
| S3_opus_medium | obs_false_fire_fraction_wholebatch | 5/10 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3_opus_medium | frac_fires_during_motion | 11/16 fires inside a motion-or-gripper call interval (pooled over runs) |
| S3_opus_medium | fire_delivery_median_s | median over 16 delivered fires pooled across runs |
| S3_opus_medium | tokens_executor | summed over 5/5 runs with a uniquely mapped executor transcript |
| S3_opus_medium | tokens_watchdog | summed over 5/5 runs with a uniquely mapped watchdog transcript |
| S4 | arming_coverage | 1191/1740 from arming_coverage over 15 runs |
| S4 | grip_coverage | 80/120 from grip_coverage over 15 runs |
| S4 | param_accuracy | 166/171 from authored_param_accuracy over 15 runs |
| S4 | verdict_correct | 13/15 = runs with verdict_correct==True / included runs |
| S4 | fault_recall | 23/35 = sum fault_TP / (sum TP + sum FN) |
| S4 | fault_precision | 23/29 = sum fault_TP / (sum TP + sum FP) |
| S4 | scorer_raw_fault_recall | 22/35 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S4 | scorer_raw_fault_precision | 22/28 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S4 | obs_engagement | 13/14 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4 | obs_detection | 0/14 = sum obs n_detected / sum obs n_detectable |
| S4 | obs_false_fire_fraction | 6/6 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4 | false_alarm_rate | 2/8 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S4 | independence_gap | 5/15 = runs with independence_gap==True / included runs |
| S4 | obs_engagement_wholebatch | 21/22 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4 | obs_detection_wholebatch | 0/22 = sum obs n_detected / sum obs n_detectable |
| S4 | obs_false_fire_fraction_wholebatch | 10/10 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4 | frac_fires_during_motion | 19/24 fires inside a motion-or-gripper call interval (pooled over runs) |
| S4 | fire_delivery_median_s | median over 24 delivered fires pooled across runs |
| S4 | tokens_executor | summed over 15/15 runs with a uniquely mapped executor transcript |
| S4 | tokens_watchdog | summed over 0/15 runs with a uniquely mapped watchdog transcript |
| S4_opus_medium | arming_coverage | 503/504 from arming_coverage over 5 runs |
| S4_opus_medium | grip_coverage | 36/36 from grip_coverage over 5 runs |
| S4_opus_medium | param_accuracy | 90/90 from authored_param_accuracy over 5 runs |
| S4_opus_medium | verdict_correct | 5/5 = runs with verdict_correct==True / included runs |
| S4_opus_medium | fault_recall | 12/14 = sum fault_TP / (sum TP + sum FN) |
| S4_opus_medium | fault_precision | 12/14 = sum fault_TP / (sum TP + sum FP) |
| S4_opus_medium | scorer_raw_fault_recall | 10/13 = sum scorer_raw_fault_TP / (sum TP + sum FN) |
| S4_opus_medium | scorer_raw_fault_precision | 10/13 = sum scorer_raw_fault_TP / (sum TP + sum FP) |
| S4_opus_medium | obs_engagement | 5/5 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4_opus_medium | obs_detection | 9/9 = sum obs n_detected / sum obs n_detectable |
| S4_opus_medium | obs_false_fire_fraction | 15/19 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4_opus_medium | false_alarm_rate | 0/2 = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs |
| S4_opus_medium | independence_gap | 1/5 = runs with independence_gap==True / included runs |
| S4_opus_medium | obs_engagement_wholebatch | 5/5 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4_opus_medium | obs_detection_wholebatch | 9/9 = sum obs n_detected / sum obs n_detectable |
| S4_opus_medium | obs_false_fire_fraction_wholebatch | 15/19 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4_opus_medium | frac_fires_during_motion | 24/29 fires inside a motion-or-gripper call interval (pooled over runs) |
| S4_opus_medium | fire_delivery_median_s | median over 29 delivered fires pooled across runs |
| S4_opus_medium | tokens_executor | summed over 5/5 runs with a uniquely mapped executor transcript |
| S4_opus_medium | tokens_watchdog | summed over 0/5 runs with a uniquely mapped watchdog transcript |


## table_cost

| row | n included | n excluded | excluded (run: reason) |
|---|---|---|---|
| S1 | 23 | 0 | - |
| S2 | 23 | 0 | - |
| S3 | 23 | 0 | - |
| S4 | 23 | 0 | - |
| S3_opus_low | 6 | 0 | - |
| S3_opus_medium | 5 | 0 | - |
| S4_opus_medium | 5 | 0 | - |

NA cells:
| row | column | reason |
|---|---|---|
| S4 | gate_overhead_fraction | no_gates_in_scenario |
| S4 | gate_overhead_fraction_ex_timeout | no_gates_in_scenario |
| S4 | gate_wait_median_s | no_gates_in_scenario |
| S4 | watchdog_postrun_adjudication_s_median | no_watchdog_role_s4_time_to_verdict_not_comparable |
| S4 | watchdog_postrun_tail_s_median | no_gates_in_scenario |
| S4_opus_medium | gate_overhead_fraction | no_gates_in_scenario |
| S4_opus_medium | gate_overhead_fraction_ex_timeout | no_gates_in_scenario |
| S4_opus_medium | gate_wait_median_s | no_gates_in_scenario |
| S4_opus_medium | watchdog_postrun_adjudication_s_median | no_watchdog_role_s4_time_to_verdict_not_comparable |
| S4_opus_medium | watchdog_postrun_tail_s_median | no_gates_in_scenario |

Numerator / denominator behind every pooled cell:
| row | column | derivation |
|---|---|---|
| S1 | gate_overhead_fraction | sum gate_wait_total_s 3737.6 / sum run_wall_s 12708.8 over 23 runs |
| S1 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 3617.6 / sum run_wall_s 12708.8 |
| S1 | gate_wait_median_s | median over 379 non-timeout gate waits pooled across runs |
| S1 | frac_fires_during_motion | 27/37 fires inside a motion-or-gripper call interval (pooled over runs) |
| S1 | fire_delivery_median_s | median over 37 delivered fires pooled across runs |
| S2 | gate_overhead_fraction | sum gate_wait_total_s 3141.2 / sum run_wall_s 13206.4 over 23 runs |
| S2 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 3141.2 / sum run_wall_s 13206.4 |
| S2 | gate_wait_median_s | median over 364 non-timeout gate waits pooled across runs |
| S2 | frac_fires_during_motion | 32/46 fires inside a motion-or-gripper call interval (pooled over runs) |
| S2 | fire_delivery_median_s | median over 44 delivered fires pooled across runs |
| S3 | gate_overhead_fraction | sum gate_wait_total_s 5084.7 / sum run_wall_s 14996.8 over 23 runs |
| S3 | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 4844.7 / sum run_wall_s 14996.8 |
| S3 | gate_wait_median_s | median over 390 non-timeout gate waits pooled across runs |
| S3 | frac_fires_during_motion | 162/178 fires inside a motion-or-gripper call interval (pooled over runs) |
| S3 | fire_delivery_median_s | median over 178 delivered fires pooled across runs |
| S4 | frac_fires_during_motion | 25/31 fires inside a motion-or-gripper call interval (pooled over runs) |
| S4 | fire_delivery_median_s | median over 31 delivered fires pooled across runs |
| S3_opus_low | gate_overhead_fraction | sum gate_wait_total_s 844.6 / sum run_wall_s 3224.1 over 6 runs |
| S3_opus_low | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 844.6 / sum run_wall_s 3224.1 |
| S3_opus_low | gate_wait_median_s | median over 97 non-timeout gate waits pooled across runs |
| S3_opus_low | frac_fires_during_motion | 5/5 fires inside a motion-or-gripper call interval (pooled over runs) |
| S3_opus_low | fire_delivery_median_s | median over 5 delivered fires pooled across runs |
| S3_opus_medium | gate_overhead_fraction | sum gate_wait_total_s 1728.1 / sum run_wall_s 3699.7 over 5 runs |
| S3_opus_medium | gate_overhead_fraction_ex_timeout | sum gate_wait_total_ex_timeout_s 1728.1 / sum run_wall_s 3699.7 |
| S3_opus_medium | gate_wait_median_s | median over 92 non-timeout gate waits pooled across runs |
| S3_opus_medium | frac_fires_during_motion | 11/16 fires inside a motion-or-gripper call interval (pooled over runs) |
| S3_opus_medium | fire_delivery_median_s | median over 16 delivered fires pooled across runs |
| S4_opus_medium | frac_fires_during_motion | 24/29 fires inside a motion-or-gripper call interval (pooled over runs) |
| S4_opus_medium | fire_delivery_median_s | median over 29 delivered fires pooled across runs |

- watchdog_blocking_cost_s is the executor's blocked time at gates only; the WatchDog's concurrent work costs no task time and is not represented. watchdog_postrun_adjudication_s is time_to_verdict_s (S1-S3). The two are separate costs and are never summed. For S4 blocking is 0 by construction.

## table_observation

| row | n included | n excluded | excluded (run: reason) |
|---|---|---|---|
| S1 | 23 | 1 | fmb2_s1c1_run04: observation_row_excluded:exclude_flag |
| S2 | 23 | 0 | - |
| S3 | 23 | 0 | - |
| S4 | 23 | 0 | - |
| S3_opus_low | 6 | 0 | - |
| S3_opus_medium | 5 | 0 | - |
| S4_opus_medium | 5 | 0 | - |
| REF | 2 | 0 | - |

NA cells:
| row | column | reason |
|---|---|---|
| S1 | obs_false_fire_fraction_wholebatch | no_observation_fires |

Numerator / denominator behind every pooled cell:
| row | column | derivation |
|---|---|---|
| S1 | obs_engagement_wholebatch | 15/19 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S1 | obs_detection_wholebatch | 0/16 = sum obs n_detected / sum obs n_detectable |
| S1 | obs_false_fire_fraction_wholebatch | 0/0 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S2 | obs_engagement_wholebatch | 10/18 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S2 | obs_detection_wholebatch | 0/18 = sum obs n_detected / sum obs n_detectable |
| S2 | obs_false_fire_fraction_wholebatch | 3/3 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3 | obs_engagement_wholebatch | 21/22 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3 | obs_detection_wholebatch | 0/22 = sum obs n_detected / sum obs n_detectable |
| S3 | obs_false_fire_fraction_wholebatch | 142/144 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4 | obs_engagement_wholebatch | 21/22 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4 | obs_detection_wholebatch | 0/22 = sum obs n_detected / sum obs n_detectable |
| S4 | obs_false_fire_fraction_wholebatch | 10/10 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3_opus_low | obs_engagement_wholebatch | 6/6 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3_opus_low | obs_detection_wholebatch | 5/9 = sum obs n_detected / sum obs n_detectable |
| S3_opus_low | obs_false_fire_fraction_wholebatch | 2/2 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S3_opus_medium | obs_engagement_wholebatch | 5/5 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S3_opus_medium | obs_detection_wholebatch | 9/10 = sum obs n_detected / sum obs n_detectable |
| S3_opus_medium | obs_false_fire_fraction_wholebatch | 5/10 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| S4_opus_medium | obs_engagement_wholebatch | 5/5 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| S4_opus_medium | obs_detection_wholebatch | 9/9 = sum obs n_detected / sum obs n_detectable |
| S4_opus_medium | obs_false_fire_fraction_wholebatch | 15/19 = sum obs_monitor_false_fires / sum obs n_observation_fires |
| REF | obs_engagement_wholebatch | 2/2 = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row |
| REF | obs_detection_wholebatch | 4/4 = sum obs n_detected / sum obs n_detectable |
| REF | obs_false_fire_fraction_wholebatch | 27/66 = sum obs_monitor_false_fires / sum obs n_observation_fires |

