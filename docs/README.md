# Documentation index

Two kinds of document live under `docs/dev/`: **procedures and references**
that are kept current, and **dated records** of how the system was built,
verified and fixed. The records are not rewritten when things change; the
current documents say what is true now.

## Current: read these to operate the system

| Document | Use it for |
|---|---|
| `dev/BATCH_RUNBOOK.md` | The ordered per-run and per-batch procedure for scored data, including the substitutions for S2 to S4, Class 4, model/effort batches and the reference oracle. |
| `dev/RUN.md` | The mechanisms behind each runbook step: session isolation, the session pointer, prerequisites, model and effort settings, launch order, vision, intruder injection, Class 4 registry corruption, permission sets. |
| `dev/SCORERS.md` | What the three scorers measure, every output column, the commands, and the hand-correction rules applied at Phase 6. |
| `dev/OBSERVATION_DUTY_AND_SCENARIOS.md` | The scene-observation duty text and the injection scenario module (`inject_scenarios.py`): record schema, behaviour, presets. |
| `../paper_results/README.md` | Layout and rules of the scored batches: raw is append-only, corrected is logged, provenance fields. |
| `../agents/README.md` | The agent harness: roles, isolation rules, the synchronization gate, transcripts, verification. |
| `../agents/tasks/runs/README.md` | The Class 2 and 3 task variants and their intended faults. |
| `../reference_watchdog/README.md` | The scripted oracle watchdog, its reference monitors, offline tests, and the live history that shaped them. |
| `../ros2_ws/BUILD_NOTES.md` | Why the workspace pins the system Python for colcon. |
| `dev/AUTHORED_PROMPT_REFS.md` | Where the prompts and task files reference the library monitor surface, for anyone editing them. |

## Records: dated, kept as written

| Document | Date | What it records |
|---|---|---|
| `dev/HANDOFF.md` | 2026-09 | Handoff after the standalone assembly planner's conventions were promoted into the live stack: the grasp-offset conventions and known test failures. |
| `dev/VISION_DISCOVERY.md` | 2026-09 | Read-only investigation ahead of adding vision. Its header lists which findings have since become false (depth is live, camera_info moved, point cloud dropped). |
| `dev/S234_codebase_report.md` | 2026-09 | Codebase survey ahead of the authored-monitor design for S2 to S4. |
| `dev/STAGE1_REPORT.md`, `dev/STAGE1_VERIFY.md` | 2026-09-30 | Stage 1: camera receive path, `look()`, first-look fixes. |
| `dev/STAGE2_REPORT.md` | 2026-09-30 | Stage 2: the authored-monitor array lane, purpose flag, sandbox and numpy fixes, wake-path fixes. |
| `dev/STAGE3_REPORT.md` | 2026-09-30 | Stage 3: the intruder injection layer and ground-truth writer. |
| `dev/STAGE4_REPORT.md` | 2026-10-01 | Stage 4: the observation duty in the prompts and task texts; the variant regeneration scheme. |
| `dev/PRE_SMOKE_VERIFY.md` | 2026-10-01 | Verification before the first smoke runs. |
| `dev/PREBATCH_FIXES.md` | 2026-10-01 | Fixes from the smoke runs before the scored batches, including TF time alignment in the recorder. |

Changes made during the scored batches are recorded where they apply: scorer
revisions in `dev/SCORERS.md` and the affected `batch.yaml` notes,
hand corrections in each batch's `corrected/CORRECTIONS.md`, and the
reference-monitor evolution in `../reference_watchdog/README.md`.
