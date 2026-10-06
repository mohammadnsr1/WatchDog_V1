# Corrections — 20261005_smoke

Every hand edit to a file in this directory gets one row, logged when the edit
is made. `raw/` is never edited. Never edit the `scorer_raw_*` columns either.

| date_utc | file | session_id | column | old value | new value | reason | by |
|---|---|---|---|---|---|---|---|
| 2026-10-05 | results.csv | *(example)* `fmb2_s1c1_20261005T1412` | `n_FP` | `1` | `0` | *(Example row, delete it when the first real row is added.)* The only FP was a grip monitor firing during the commanded release at place. Replaying the bag shows the gripper-open command was sent just before the fire, so the fire was expected behavior, not a false alarm. `FP_list` and `precision` were corrected in the same pass. | MN |
