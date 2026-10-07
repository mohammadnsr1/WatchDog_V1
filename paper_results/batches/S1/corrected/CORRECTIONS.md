# Corrections — S1

Every hand edit to a file in this directory gets one row, logged when the edit
is made. `raw/` is never edited. Never edit the `scorer_raw_*` columns either.

## Exclusions to apply when corrected/ is built (Phase 6)

Decided 2026-10-07. `raw/` is append-only, so these rows stay there; in the
`corrected/` copy each gets `exclude=1`, and a row goes in the table below
when that edit is made.

`raw/results.csv` (keyed by run_id):

| raw row | run_id | session_id | reason |
|---|---|---|---|
| 1 | `fmb1_s1c1_run01` | 20261007T205057Z | void run (`known_invalid`); also wrong label (fmb1 template) |
| 2 | `fmb1_s1c1_run02` | 20261007T213533Z | wrong label (fmb1 template); superseded by `fmb2_s1c1_run01` (rescored on f67ff37) |
| 3 | `fmb1_s1c1_run03` | 20261007T215407Z | wrong label; superseded by `fmb2_s1c1_run02` |
| 4 | `fmb1_s1c1_run04` | 20261007T220959Z | wrong label; superseded by `fmb2_s1c1_run03` |

`raw/observation_results.csv` (keyed by session_id; row = data row number):

| raw row | session_id | reason |
|---|---|---|
| 1 | 20261007T205057Z | void run (`known_invalid`) |
| 2 | 20261007T213533Z | scored before f67ff37; superseded by row 6 |
| 3 | 20261007T215407Z | scored before f67ff37; superseded by row 7 |
| 4 | 20261007T220959Z | scored before f67ff37 (placement settle counted as a missed tracked_move); superseded by row 8 |
| 5 | 20261007T220959Z | duplicate of row 4; superseded by row 8 |

## Hand edits

| date_utc | file | session_id | column | old value | new value | reason | by |
|---|---|---|---|---|---|---|---|
