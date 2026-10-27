# Label templates

The per-run ground-truth label is the operator's record of what physically
happened, written from the bag and from watching the run, never from the
agents' claims. The scorers read it (`watchdog_scorer.py` and `s4_scorer.py`
need it; `observation_scorer.py` reads the injection ground truth instead).

Copy the closest template into the session directory and fill in every
per-run field, as described in `docs/dev/BATCH_RUNBOOK.md` Phase 3:

| Template | Assembly | Scenario | Class |
|---|---|---|---|
| `gt_fmb1_s1c1.yaml` | fmb_assembly | S1 | 1, clean |
| `gt_fmb2_s1c1.yaml` | fmb_assembly_2 | S1 | 1, clean |
| `gt_fmb2_s1c2.yaml` | fmb_assembly_2 | S1 | 2, injected or natural fault |
| `gt_fmb2_s1c4.yaml` | fmb_assembly_2 | S1 | 4, corrupted specification |

For other scenarios change `scenario:` and `run_id:`; the keys the scorer
reads are listed in each template's header comment. Exactly one `gt_*.yaml`
per session directory.
