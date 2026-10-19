# Corrections — S1

Every hand edit to a file in this directory gets one row, logged when the edit
is made. `raw/` is never edited. Never edit the `scorer_raw_*` columns either.

## Hand edits

| date_utc | file | session_id | column | old value | new value | reason | by |
|---|---|---|---|---|---|---|---|
| 2026-10-19 | results.csv | *(all rows)* | `fault_TP_by_fire`, `fault_TP_by_check` | *(column absent)* | recomputed per run | corrected/ initialised as a copy of raw/ (Phase 6 step 1; observation_results.csv unchanged), then the two additive columns from the 2026-10-19 scorer revision written by `scripts/backfill_tp_mode.py`. No existing cell changed (verified cell by cell against raw/); each row filled only where the recomputed fault_TP equals the CSV's. | Claude for MN |
