"""Read-only analysis layer over the WatchDog scored batches.

Turns the scored batches under ``paper_results/batches`` plus the raw session
logs under ``agents/transcripts/sessions`` into the paper tables and a per-run
artifact of record.  Pure functions, no global state, no network access.  It
never modifies the scorers or their outputs.

Entry point: ``python3 -m analysis --out out``.
"""
