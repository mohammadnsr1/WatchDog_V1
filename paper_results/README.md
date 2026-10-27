# paper_results

This directory holds the scorer output that the paper's numbers are computed
from, grouped by batch. It holds scorer output and session IDs only. The
sessions themselves (rosbags, camera images, event logs, MCP call logs) stay
wherever the run wrote them and never enter this tree or git. The
`.gitignore` here drops the common session artifacts if one is copied in by
mistake, but it is a safety net, not a place to keep them.

## Layout

```text
paper_results/
  README.md
  .gitignore
  batches/
    <batch_id>/              e.g. 20261005_smoke, 20261012_scored_sonnet_low
      batch.yaml             provenance, models, and the list of sessions
      raw/                   scorer output exactly as written
      corrected/             hand-corrected copies of the raw CSVs
        CORRECTIONS.md       log of every hand edit
      summary/               output of --summarize over corrected/
  tables/                    paper-facing sources for Tables I-V
```

A batch ID is the UTC date the batch was run followed by a short description,
for example `20261012_scored_sonnet_low`. A batch is either a smoke batch,
which checks that the pipeline works and never feeds a paper number, or a
scored batch, which does. `batch.yaml` records which one it is.

## Batches

| batch_id | agents | scenario | runs | note |
|---|---|---|---|---|
| `20261005_smoke` | Sonnet-5 low | S1 | smoke | pipeline check, never scored |
| `S1`, `S2`, `S3` | Sonnet-5 low / Sonnet-5 low | S1, S2, S3 | 23 each (C1 5, C2 10, C3 5, C4 3) | the baseline |
| `S4` | Sonnet-5 low executor, no watchdog | S4 | 23 | the baseline, self-monitoring |
| `S3_opus_low` | Opus-5.5 low watchdog / Sonnet-5 low executor | S3 | 6 (C1 5, C2 1) | capability study; hand visit added from run 03 |
| `S3_opus_medium` | Opus-5.5 medium watchdog / Sonnet-5 low executor | S3 | 5 (C1 4, C2 1) | capability study; cube + hand every run |
| `S4_opus_medium` | Opus-5.5 medium executor, no watchdog | S4 | 5 (C1 1, C2 4) | capability study; cube + hand every run |
| `REF` | scripted oracle watchdog / Sonnet-5 low executor | S3 protocol | 2 | observation upper bound; observation-scored only |

Opus batches ran Class 1 and 2 only, so conformance comparisons against the
baseline use the baseline's Class 1 and 2 rows. Each `batch.yaml` records the
design, every session, the stage and rule changes that happened during the
batch, and the sessions excluded as `known_invalid`.

## Which scorer writes which file

Each scorer writes one JSON file per session and appends one row per session
to a CSV.

- `watchdog_scorer.py` writes `score.json` and appends to `results.csv`.
- `s4_scorer.py` writes `score_s4.json` and appends to `results_s4.csv`.
- `observation_scorer.py` writes `observation_score.json` and appends to
  `observation_results.csv`.

Inside `raw/`, the CSVs sit at the top level, one per scorer, and each
session's JSON files go in a subdirectory named after the session ID,
`raw/<session_id>/`. The JSON names are fixed, so they would overwrite each
other if they were kept flat.

Each scorer always writes its JSON into the session directory as well. Use
`--csv` and `--out` to make the scorer itself write into `raw/`, so nothing
has to be copied by hand. Missing directories are created. For example:

```bash
B=paper_results/batches/<batch_id>/raw
python3 watchdog_scorer.py <session_dir> --csv $B/results.csv --out $B/<session_id>/score.json
python3 s4_scorer.py <session_dir> --csv $B/results_s4.csv --out $B/<session_id>/score_s4.json
python3 observation_scorer.py <session_dir> --scenario S2 \
    --csv $B/observation_results.csv --out $B/<session_id>/observation_score.json
```

## raw/ is append-only

`raw/` holds scorer output exactly as the scorers wrote it. Nobody edits a
file in `raw/` by hand, for any reason. New sessions are added by running the
scorer again, which appends a row. If a raw value is wrong, the fix goes in
`corrected/`, never in `raw/`. Keeping `raw/` untouched means any number in the
paper can be traced back to what the scorer actually produced.

## corrected/ is what gets summarized

`corrected/` holds hand-corrected copies of the raw CSVs. To start, copy each
raw CSV into `corrected/` unchanged, then make any corrections in the copy.
`watchdog_scorer.py --summarize` is run only on the CSVs in `corrected/`,
never on `raw/`, and its output goes in `summary/`.

Every edit to a file in `corrected/` must be logged in
`corrected/CORRECTIONS.md`, with the session ID, the column, the old value, the
new value and the reason. An edit that is not in the log is treated as an
error. In `results.csv`, the `scorer_raw_*` columns mirror the machine's values
and are never edited, even in `corrected/`. They exist so a reader can see what
was changed.

To exclude a session from the paper numbers, list it under `known_invalid` in
`batch.yaml` with the reason, and set its `exclude` column to `1` in the
corrected CSV. `--summarize` drops rows marked `exclude`. Log the change in
`CORRECTIONS.md` like any other edit. `--summarize` does not remove duplicate
rows. If a session was scored twice and appears twice in a raw CSV, mark one
of the rows excluded in the corrected copy and log it.

## Run the leak audit before the observation scorer

For any session that had intruders injected, run
`scripts/audit_intruder_leak.py <session_dir>` before running
`observation_scorer.py` on it. The audit writes `intruder_leak_audit.json`
into the session directory, and the observation scorer reads it to decide
whether the session is excluded. If the audit has not been run, the
observation scorer still produces a row, but the row's `leak` value is blank
and it carries a warning. That row cannot be used in the paper.

## Provenance

`batch.yaml` records `git_sha` and `registry_sha256`. The registry hash is
taken from the install-space copy,
`ros2_ws/install/robot_backend/share/robot_backend/config/model_registry.yaml`,
because that is the file a live run loads. The source copy can differ from it
if `colcon build` was not rerun after an edit. This is the clean registry for
the batch.

Class 4 runs hand the EXECUTOR a corrupted registry via the
`EXECUTOR_REGISTRY_PATH` override (built with
`scripts/make_corrupt_registry.py`; see RUN.md's Class 4 section). The cell
and, in S1–S3, the WatchDog keep loading the clean install-space registry —
so the session's `manifest.json` records the CLEAN hash on a Class 4 run.
The hash of what the executor actually loaded is written by its servers to
the session's `registry_provenance_executor.json`, and that is the value that
goes in the session's `run_registry_sha256` field in `batch.yaml` (the
corruption script prints it). `s4_scorer.py` reads the same file first
(manifest as fallback), scores thresholds against the true install-space
registry by default, records both hashes in every row
(`scored_registry_sha256`, `run_registry_sha256`, `run_registry_source`,
`registry_differs_from_run`), and refuses to score a Class 4 run if the two
match, because the corruption would then be invisible.
