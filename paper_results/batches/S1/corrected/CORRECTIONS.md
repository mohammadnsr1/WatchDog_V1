# Corrections — S1

Every hand edit to a file in this directory gets one row, logged when the edit
is made. `raw/` is never edited. Never edit the `scorer_raw_*` columns either.

## Hand edits

| date_utc | file | session_id | column | old value | new value | reason | by |
|---|---|---|---|---|---|---|---|
| 2026-10-19 | results.csv | *(all rows)* | `fault_TP_by_fire`, `fault_TP_by_check` | *(column absent)* | recomputed per run | corrected/ initialised as a copy of raw/ (Phase 6 step 1; observation_results.csv unchanged), then the two additive columns from the 2026-10-19 scorer revision written by `scripts/backfill_tp_mode.py`. No existing cell changed (verified cell by cell against raw/); each row filled only where the recomputed fault_TP equals the CSV's. | Claude for MN |
| 2026-10-23 | observation_results.csv | `20261007T232107Z` | `exclude` | `(blank)` | `1` | fmb2_s1c1_run04: the injection script was started late, so the cube spawned at 23:27:31 during place_line_green instead of at the pinned moment (pick_u_orange, 23:24:14). Decided in batch.yaml at scoring time; conformance row untouched. | Claude for MN |
| 2026-10-23 | observation_results.csv | `20261007T232107Z` | `manual_note` | `(blank)` | `late cube injection (spawned during place_line_green; not at pick_u_orange); excluded from observation numbers per batch.yaml` | fmb2_s1c1_run04: the injection script was started late, so the cube spawned at 23:27:31 during place_line_green instead of at the pinned moment (pick_u_orange, 23:24:14). Decided in batch.yaml at scoring time; conformance row untouched. | Claude for MN |
