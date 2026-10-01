# Stage 4 — scene observation: prompt and task text (2026-10-01)

Text edits to the S1 sources plus builder changes, with every derived file
regenerated mechanically. No colcon, no servers, no Isaac. Principle held to:
the added text states capabilities and requirements only — no monitoring
strategy, no thresholds, no pixel/region hints, no scene-specific objects or
coordinates anywhere in the additions.

Line numbers below are from the files AS READ before editing.

## 0. Read-first findings

**watchdog_system.txt** (646 lines pre-edit)
- "The tools" section: lines 73–277. `capture_frames` entry at 165–174; the
  "You cannot see these frames" wording at 169–171 — the only
  scene-can't-see statement in the prompt. (executor_system.txt:304 "the
  WatchDog cannot see you think" is about cognition, not the scene; left in
  the S1 executor prompt, and G_REPLACE already rewrites it for S4.)
- Evidence/threshold discipline, location 1: the "Every threshold you use
  must come from a contract clause or a registry spec field…" paragraph at
  51–55 (§Evidence discipline, 49–71).
- Location 2 (the task's "~597"): the Failure modes entry at line 606,
  "Using a threshold you computed rather than read from the specification."
  Nothing threshold-related sits at 597 itself; 606 is the rule the
  exemption qualifies.
- Unplanned-step "name it, do not flag it" rule: §"Step recognition and
  coverage are different things", 319–349 ("Do NOT flag it" at 331–334,
  "you MUST name it" at 340–345), ending just before "## Control loop" (351).
- Run-verdict cadence: §5a at 486–551; the instruction to record the run
  verdict is "Then finalize. You ALWAYS call record_run_verdict exactly
  once…" at 505–507. (Restated in §Verdicts at 586–602.)

**prompt_variants.py** (455 lines pre-edit)
- Ops: A_DELETE (294), B_DELETE (301, alt end anchor logic 199–207),
  C_INSERT (308), D_REPLACE (312), E_INSERT (316), F_INSERT/G_REPLACE/
  H_REPLACE (322–333). Blocks: BLOCK_C (24), BLOCK_E (63), D_OLD/D_NEW
  (68/74), _S4_HEAD (80), _S4_TAIL (108), S4_SECTION (159), G/H texts
  (161–186). SCENARIO_OPS (350): S2=[C,E], S3=[A,B,C,D,E].
- s4_task_expected (417–446); its Effects–Invariants slice anchors:
  WD_REQ_START_PREFIX="Effects (must hold" (211),
  WD_REQ_END_PREFIX="Preconditions:" (212). **The slice already ends at the
  "Preconditions:" line**, so text inserted after the Invariants block and
  before Preconditions is included automatically — no anchor extension was
  needed (task item 3's conditional did not trigger).
- test_prompt_variants.py pins every variant byte-for-byte to the builders
  and every anchor to exactly-once (lines 61–123), and the S4 task files to
  s4_task_expected (156–198).

**executor_system.txt** (317 lines pre-edit)
- "Your tools" section: 13–19 — PROSE ONLY, no per-tool entries existed.
- Step 2 at 212–223: "Read the contract. Retrieve the contract for the
  object… through the contract tools." HANDOFF.md:175 confirms the executor
  **cannot call get_contract** (watchdog-only server). Real equivalents
  verified: `get_object_spec` (scene_mcp_server.py:423, registry spec per
  object) and `get_rendered_task` (motion_mcp_server.py:466, the task text
  incl. the steps: block). Step 2's text is NOT pinned by any guard test;
  the fix flows to S4 via the builder.

**Task yamls**
- fmb_assembly.watchdog.yaml: Invariants block 29–43, blank 44,
  "  Preconditions:" 45. fmb_assembly_2.watchdog.yaml: Invariants 30–44,
  "  Preconditions:" 46. Both instruction_spec block scalars use a uniform
  2-space indent, which yaml strips, so WD_REQ anchors match at column 0 of
  the parsed text.

## 1. Edits to watchdog_system.txt (S1 source; flows to S2/S3)

- **1a** — two new tool entries beside capture_frames, in the section's
  name-line + 4-space-body format: `look(...)` inserted immediately BEFORE
  capture_frames, `record_observation(...)` immediately AFTER it. Passages
  verbatim as specified.
- **1b** — capture_frames description replaced wholesale (signature line
  kept) with the verbatim two-sentence text. The old description's
  buffer-recency note was part of the replaced description and is gone from
  the entry; the recency point survives at the control-loop mention ("the
  buffer will not hold it indefinitely", step 4). No other "cannot see the
  scene" sentence existed (see step 0).
- **1c** — the exemption inserted verbatim at BOTH locations: as its own
  paragraph directly after the 51–55 rule, and attached as continuation
  lines to the Failure modes threshold entry (line 606 pre-edit). It
  therefore appears **twice** per watchdog prompt; the guard test asserts
  exactly 2.
- **1d** — the scene-change passage inserted verbatim as its own block
  between the end of the unplanned-step section (349) and "## Control loop"
  (351). No `##` heading added: the passage's own lead-in phrase
  ("Unexpected changes in the scene.") serves as the subsection header,
  keeping the text strictly verbatim.
- **1e** — the close-out sentence inserted as its own (4-space-indented, to
  match §5a) paragraph immediately before "Then finalize. You ALWAYS call
  record_run_verdict exactly once…".

## 2. S2/S3-only builder op

- New `BLOCK_I` + `_op_i_insert` ("I_INSERT") in prompt_variants.py.
  Anchored before the disarm line and ordered after C_INSERT, so the
  sentence lands immediately after BLOCK_C and reads as the closing
  paragraph of the arm_custom_monitor entry (4-space indented). Applied by
  SCENARIO_OPS for S2 and S3, never S1. The S3-ablation identity
  (S3 == S2 minus A/B/D) still holds because B_DELETE's alternate end
  anchor stops at BLOCK_C, leaving BLOCK_I intact — test passes unchanged.
- Guard test `test_camera_monitor_sentence_only_in_authored_scenarios`:
  exactly once in S2, S3 (and S4); zero in both S1 prompts.

## 3. Watchdog task files

The "Workspace observation:" requirement inserted VERBATIM (2-space block
indent) between the Invariants block and the "Preconditions:" line in both
fmb_assembly.watchdog.yaml and fmb_assembly_2.watchdog.yaml. Because the S4
slice already ends at "Preconditions:", the requirement flows into both S4
executor tasks with no builder change; regenerated, it appears exactly once
in each (guard-tested), and zero times in the S1–S3 executor tasks.

## 4. Executor prompts

- **4a** — look entry added to executor_system.txt after the "Your tools"
  paragraph, in the watchdog prompt's entry format (the executor section had
  no per-tool format of its own — that is the one formatting adaptation).
  No observation duty added for S1–S3 executors.
- **4b** — step 2 before/after:

  Before (212–223): "2. Read the contract. Retrieve the contract for the
  object this step concerns. Its parameters … sourced from the cell's
  registry through the contract tools. … A contract is static per-object
  reference data, so fetch each object's once … they come from this step's
  entry in the task instruction's steps: block, and you pass them…"

  After: "2. Read the object specification. Retrieve it for the object this
  step concerns with get_object_spec. Its parameters … sourced from the
  cell's registry. … An object specification is static per-object reference
  data, so fetch each object's once … they come from this step's entry in
  the task instruction's steps: block, already rendered into your task text
  by get_rendered_task, and you pass them…"

  Plus one adjacent consistency fix in the tools paragraph (line 16–17):
  "the object contracts are your source of truth" → "the object
  specifications are your source of truth". No guard test pins step 2, so
  this is a direct S1-source edit; S4 picks it up via regeneration.
- **4c** — S4 additions, all through prompt_variants.py blocks:
  - record_observation entry (1a text, unadapted — it was already
    role-neutral) added to _S4_TAIL between the evidence tools and
    record_verdict;
  - 1c exemption (verbatim) added to _S4_HEAD after its threshold-discipline
    paragraph — once, since S4 has one threshold-rule location;
  - 1d subsection added at the end of _S4_TAIL, adapted ONLY for role:
    "while the executor works" → "while you work" (the S4 executor is the
    one moving the arm); the rest verbatim;
  - 1e sentence (verbatim) appended as the closing paragraph of _S4_TAIL's
    record_run_verdict entry ("call exactly once, immediately before
    end_run" is that entry's own first line, so the close-out sits with the
    instruction it qualifies);
  - section-2 sentence: BLOCK_I embedded in _S4_TAIL right after the
    raw_spec note, closing the arm_custom_monitor entry exactly as I_INSERT
    places it in S2/S3;
  - the look entry reaches S4 automatically from executor_system.txt.
  - The builder strips no comments, so the "record_observation on the
    executor side awaits the S4 monitor host" mark went to **RUN.md's
    scenario table (S4 row)**, per the task's either/or.

## 5. Derived vs hand-edited; regeneration

Hand-edited (S1 sources + builder): watchdog_system.txt,
executor_system.txt, fmb_assembly.watchdog.yaml,
fmb_assembly_2.watchdog.yaml, prompt_variants.py,
test_prompt_variants.py (new guard tests appended).

Regenerated, never hand-edited: watchdog_system_s2.txt,
watchdog_system_s3.txt, executor_system_s4.txt (builder output byte for
byte), fmb_assembly_s4.executor.yaml and fmb_assembly_2_s4.executor.yaml
(only their instruction_spec / object_ids block scalars rewritten to
s4_task_expected's values; comments and steps: untouched).

New: **agents/prompts/regenerate_variants.py** — the previously missing
mechanical regeneration step (the S4 task files had been hand-maintained
against the pinning test; one trailing-space line in
fmb_assembly.executor.yaml is preserved verbatim by the script's
round-trip check).

## 6. Test results

```
test_prompt_variants (24 pre-existing + 4 new guard tests)
+ test_scenario_gating + test_observation            -> 55 passed
STAGE1_VERIFY §2 targeted list (test_run_ended_terminal_event,
 test_session_and_image_buffer, test_threshold_provenance,
 test_authored_monitor, test_authored_lane, test_authored_parity,
 test_recorder_windows) + Stage 3 tests (test_inject_intruder,
 test_audit_intruder_leak)                           -> 141 passed
```

New guard tests assert, by whitespace-normalised count: 1a/1b/1d/1e exactly
once and 1c exactly twice in all three watchdog prompts; BLOCK_I once in
S2/S3/S4 and zero in both S1 prompts; the adapted passages once each in the
S4 executor prompt and absent from the S1 executor (which carries only the
look entry); the workspace requirement once in both watchdog tasks and both
S4 executor tasks, zero in the S1–S3 executor tasks.

## 7. Unresolved / notes

- The S4 executor prompt now documents record_observation, but the tool does
  not exist on any executor-reachable server yet — intentional, flagged in
  RUN.md; the S4 monitor host remains the open Stage 3 item.
- executor_system.txt line ~304 still says "the WatchDog cannot see you
  think" in the S1 text — correct for S1–S3 (there IS a WatchDog) and
  rewritten for S4 by the existing G_REPLACE.
- The capture_frames buffer-recency guidance now lives only in the control
  loop (step 4), per 1b's wholesale replacement.
