# Pre-smoke verification (2026-10-01)

Report only; nothing was edited except this file. Everything ran with
`ros2_ws/.venv/bin/python3` under `unset VIRTUAL_ENV`, ROS Jazzy + install
space sourced, `_ROBOT_BACKEND_MCP_VENV_READY=1`,
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. No colcon, no ros2 CLI, no launch, no
Isaac, no servers.

**Verdict: clear to smoke-test, with three things to know first** —
(a) the `~/.watchdog/current_session` pointer is currently dangling, so both
sessions would hard-fail until the bringup rewrites it (expected pre-bringup,
but it is the first thing that will stop you); (b) the intruder scripts'
example/default spawn heights are ~0.76–0.80 m **above** the bench, which is
world Z = 0, not 0.76 (§5); (c) the scorer is blind to authored monitors AND
would count an observation fire as a monitor false alarm (§4).

## 1. Rendered prompts

All ten combinations rendered successfully through
`role_context.build_rendered_task`, using each `*.mcp.json`'s own env. No path
rewriting was needed: every checked-in mcp.json already points at this
checkout (`/home/mohammad/watchdog/WatchDog_V1/...`). The S4 executor has **no
mcp.json of its own** (there is no `executor_s4.mcp.json`), so its env was
synthesized from `agents/config/executor_agent_s4.yaml` + the `*_s4.executor.yaml`
task — consistent with S4 not being runnable yet.

`WATCHDOG_SESSION_DIR` was pointed at a scratch directory for these renders,
because `resolve_session_dir_strict()` refuses rule='fresh' and the pointer is
currently dangling (see §1.4).

### 1.1 Source tree vs install space — **no colcon needed for prompt edits**

| Field | Resolves to |
|---|---|
| `sources.system_prompt_path` | `agents/prompts/*.txt` — **source tree** |
| `sources.task_path` | `agents/tasks/*.yaml` — **source tree** |
| `sources.agent_config` | `agents/config/*.yaml` — **source tree** |
| `sources.registry_path` | `ros2_ws/install/.../share/robot_backend/config/model_registry.yaml` — **install space** |

Mechanism: `role_context._agents_dir()` walks up from `__file__` looking for a
parent holding both `agents/` and `ros2_ws/`. Because the install space lives
*inside* the repo (`ros2_ws/install/...`), that walk reaches the same repo root
whether the module was imported from source or from install space. So **prompt,
task and config edits reach a live session immediately — a colcon build is NOT
required for them.**

The registry is the opposite: `_live_registry_provenance()` resolves install
space first (via `get_package_share_directory`), so **registry edits still
require a colcon build**, exactly as the RUN.md pre-run checklist step 1 says.
Checked now: install and source registries are identical
(sha256 `ff4a6f968a9192f2…` both), so checklist step 4 currently passes.

### 1.2 Stage 4 passages — present where they belong, absent where they do not

Counts are whitespace-normalised, reusing the guard-test passage constants from
`test_prompt_variants.py` (so wrapping differences between a prompt and a YAML
block do not mask a match). Each cell is `prompt,task`.

| Combo | look(wd) | look(exec) | record_obs | capture | thresh-exempt | scene(wd) | scene(s4) | scene-tail | closeout | workspace-req |
|---|---|---|---|---|---|---|---|---|---|---|
| watchdog S1 fmb_assembly | 1,0 | 0,0 | 1,0 | 1,0 | 2,0 | 1,0 | 0,0 | 1,0 | 1,0 | 0,1 |
| watchdog S2 fmb_assembly | 1,0 | 0,0 | 1,0 | 1,0 | 2,0 | 1,0 | 0,0 | 1,0 | 1,0 | 0,1 |
| watchdog S3 fmb_assembly | 1,0 | 0,0 | 1,0 | 1,0 | 2,0 | 1,0 | 0,0 | 1,0 | 1,0 | 0,1 |
| watchdog S1 fmb_assembly_2 | 1,0 | 0,0 | 1,0 | 1,0 | 2,0 | 1,0 | 0,0 | 1,0 | 1,0 | 0,1 |
| watchdog S2 fmb_assembly_2 | 1,0 | 0,0 | 1,0 | 1,0 | 2,0 | 1,0 | 0,0 | 1,0 | 1,0 | 0,1 |
| watchdog S3 fmb_assembly_2 | 1,0 | 0,0 | 1,0 | 1,0 | 2,0 | 1,0 | 0,0 | 1,0 | 1,0 | 0,1 |
| executor fmb_assembly | 0,0 | 1,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 |
| executor fmb_assembly_2 | 0,0 | 1,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 | 0,0 |
| executor S4 fmb_assembly | 0,0 | 1,0 | 1,0 | 0,0 | 1,0 | 0,0 | 1,0 | 1,0 | 1,0 | 0,1 |
| executor S4 fmb_assembly_2 | 0,0 | 1,0 | 1,0 | 0,0 | 1,0 | 0,0 | 1,0 | 1,0 | 1,0 | 0,1 |

Every cell matches the Stage 4 design:

- the watchdog look entry, record_observation, the capture_frames replacement,
  the scene-change passage and the close-out appear **exactly once** in all six
  watchdog renders; the threshold exemption appears **twice** by design (once
  in Evidence discipline, once in Failure modes);
- the S1–S3 executor carries **only** the look entry and no observation duty;
- the S4 executor carries the role-adapted scene passage (`scene(s4)`, "while
  you work") and never the watchdog wording (`scene(wd)` = 0);
- the workspace requirement appears once in each watchdog task and each S4
  executor task, and **zero** times in the S1–S3 executor tasks.

The camera-capability sentence (`BLOCK_I`, checked separately since it is a
builder constant rather than a `P_*` passage): 0 in `watchdog_system.txt`, 0 in
`executor_system.txt`, **1** in `watchdog_system_s2.txt`,
`watchdog_system_s3.txt` and `executor_system_s4.txt`.

**No `{{PLACEHOLDER}}` survives in any of the twenty rendered parts.**
(`render_task` hard-fails on one, and none fired.)

### 1.3 Part sizes against the CLI result cap

The WatchDog fetches the two fields separately (`get_rendered_task(part=...)`),
so the cap applies per part. Sizes below are the raw text and the same text
JSON-wrapped as the tool result actually ships it.

| Part | raw bytes | JSON-wrapped | vs ~45 KB |
|---|---|---|---|
| watchdog_system.txt (S1) | 39,115 | 39,931 | ok |
| **watchdog_system_s2.txt** | **41,803** | **42,669** | **ok — closest to the line (2.3 KB headroom)** |
| watchdog_system_s3.txt | 40,625 | 41,471 | ok |
| executor_system.txt | 18,750 | 19,211 | ok |
| executor_system_s4.txt | 26,582 | 27,188 | ok |
| task, fmb_assembly (watchdog) | 11,517 | — | ok |
| task, fmb_assembly_2 (watchdog) | 11,136 | — | ok |
| task, fmb_assembly (executor) | 9,173 | — | ok |
| task, fmb_assembly_2 (executor) | 8,658 | — | ok |
| task, fmb_assembly S4 | 13,114 | — | ok |
| task, fmb_assembly_2 S4 | 12,593 | — | ok |

**Nothing is over 45 KB.** The S2 watchdog system prompt is the largest part at
42.7 KB wrapped; Stage 4 added ~2.7 KB to it, so roughly one more Stage-4-sized
addition would reach the flag threshold. (The tool also carries
`anthropic/maxResultSizeChars: 400_000`, so the hard limit is far higher; 45 KB
is the conservative line this check uses.)

### 1.4 Incidental finding — the session pointer is dangling

`~/.watchdog/current_session` names
`agents/transcripts/sessions/20261001T022555Z`, which no longer exists
(`agents/transcripts/sessions/` is empty — the post-leak-audit session was
deleted). `resolve_session_dir_strict()` raises on rule='fresh', so
**`get_rendered_task` would hard-fail in both sessions until the bringup
rewrites the pointer.** That is the designed behaviour and the bringup fixes
it, but it means the very first tool call of a smoke run fails if the cell is
not up first.

## 2. Derived files are up to date

`regenerate_variants.py` reported `unchanged` for all five derived files:

```
unchanged  watchdog_system_s2.txt
unchanged  watchdog_system_s3.txt
unchanged  executor_system_s4.txt
unchanged  fmb_assembly_s4.executor.yaml
unchanged  fmb_assembly_2_s4.executor.yaml
```

The script is idempotent and write-skipping, so this run was effectively a dry
run: nothing on disk changed. `test_prompt_variants.py` independently pins the
same equality byte for byte, and passes.

(The `git diff --stat` on those files is non-empty, but that is the uncommitted
Stage 4 work versus HEAD, not builder drift.)

## 3. Full test suite — no new failures

```
12 failed, 527 passed, 1 skipped, 1 error (collection) in 23.29s
```

To separate "known-broken" from "new", the same suite was run against a clean
tree (`git stash -u`, then restored):

```
clean HEAD:      12 failed, 523 passed, 1 skipped, 1 error
with Stage 4:    12 failed, 527 passed, 1 skipped, 1 error
```

**Identical failure set; +4 passing tests, which are exactly the four Stage 4
guard tests.** Nothing new.

The 12 + 1, by family:

| Count | Tests | Known? |
|---|---|---|
| 2 | `test_flake8`, `test_pep257` | known lint pair |
| 5 | `test_pick_contract.py::TestLoadFromRegistry::*` (cube force/effort values) | known contract/registry family |
| 4 | `test_registry_loader.py::test_m0/m1/m4/test_contract_and_gripper_consumers…` | known contract/registry family |
| 1 | `test_rg2_geometry.py::test_width_lookup_round_trip_matches_requested_width` | known rg2 round-trip |
| 1 error | `test_grasp_skill.py` collection (`AIR_CLOSE_FLOOR_EFFORT` gone) | known |

One bookkeeping note: HANDOFF §6 describes the known set as "15 failures: 6
event-coverage, 6 contract/registry, 1 grasp_skill, 1 rg2, 2 lint". The
composition here differs legitimately — the **6 event-coverage failures do not
occur** under the venv interpreter (§6 attributes them to "FastMCP not
importable in system python", and the venv has FastMCP), and the
contract/registry family counts **9** rather than 6 under the current test
files. Both match the clean-tree baseline exactly, so neither is attributable
to Stage 3 or Stage 4. **HANDOFF §6's numbers are stale against the documented
venv recipe** and would be worth restating as "12 failed + 1 collection error"
next time that file is touched.

## 4. Scorer readiness — not ready, in three distinct ways

Read-only review of `watchdog_scorer.py` (737 lines).

**4a. Authored monitors are invisible to coverage and provenance.** The
dispatch table is library-only:

```python
FAMILY = {"stationarity": …, "relative_frame": …, "effort_event": …, "region": …}
```

An authored monitor arms with `monitor_type = "authored"`
(`authored.py:793`, `super().__init__("authored", …)`), so `FAMILY.get("authored")`
is `None`, and:

- **Coverage** (the arm/disarm interval builder): every branch is guarded by
  `if fam and po:` — with `fam = None` the monitor's live window is **never
  recorded**, so an authored monitor covering a requirement scores as a
  coverage **miss**. An S3 run (authored only) would score ~0 coverage by
  construction.
- **Provenance**: the loop does
  `if obj in FIXTURES or src == "object_unresolved" or not fam.get("reg_field"): continue`
  — authored arms are skipped out of the denominator entirely, so an authored
  monitor armed from a genuine registry value is neither credited nor faulted.

**4b. A `purpose="observation"` fire would be counted as a monitor FALSE
ALARM.** The scorer never reads `purpose` — the string does not appear in the
file. The only authored-fire special case is `_is_probe()`, which keys on
`origin == "authored" and triggering_value == 0.0`. An observation fire carries
a real positive magnitude (e.g. the changed-pixel fraction), so it is *not* a
probe, and on a run with `violation_occurred: false` it lands in
`monitor_false_alarms`. Answering the question precisely:

| Situation | How an observation fire is counted today |
|---|---|
| Clean run (`violation_occurred: false`) | **False positive** — counted in `monitor_false_alarms` |
| Clean run, fire value exactly 0.0 | Filed as `authored_probe_fires` (excluded from false alarms) |
| Violation run | `monitor_false_alarms` is `None`; but the fire's `object_id` is folded into `finding_pairs` for any step that also carries a finding verdict, so it can **manufacture a detection false positive** against an object the agent never faulted |

This is the inverse of the intended semantics: an observation fire is a
look-trigger and should be **ignored** by both the false-alarm and detection
metrics (the event stream already marks it, `purpose == "observation"`, on both
arm and fire payloads).

**4c. It reads nothing from the Stage 3 observation layer.** No occurrence of
`observation`, `record_observation`, `ground_truth`, `intruder`, `leak` or
`purpose` anywhere in the scorer. It does not read `observation` events, does
not read `ground_truth_injections.jsonl`, and does not consult
`intruder_leak_audit.json` — so the `leak: true` exclusion rule is currently
**manual only** (which is what RUN.md's new checklist row says).

**4d. Is RUN.md's warning still accurate?** Yes, and it is now understated.
RUN.md says "S2/S3 runs may be collected now but must not be scored until the
scorer recognises authored monitors" — still true (4a). But observation
scoring needs three further changes: skip `purpose == "observation"` fires in
the false-alarm and detection blocks (4b), read observation events against
`ground_truth_injections.jsonl` (4c), and honour the leak audit. Suggested
wording for the next RUN.md edit: *"S2/S3 runs, and any run with injected
intruders, may be collected now but must not be scored until the scorer
recognises authored monitors, ignores observation-purpose fires, and reads the
observation/ground-truth pair."*

## 5. Bench height — the example z is wrong by the full bench height

**The bench top is world Z = 0, not 0.76.** Three independent sources:

- `agents/prompts/watchdog_system.txt:29` and
  `agents/prompts/executor_system.txt:80`, identical wording: *"The bench top
  the parts rest on is world Z = 0, so a part sitting on the bench has a small
  positive Z, and 'lift' means increasing world Z."*
- `sim_moveit_bringup.launch.py:850-854`, the `workspace_floor_top_z` argument,
  `default_value="-0.05"`, described as *"Top face of the slab in the target
  frame. The bench top is world Z=0…"*. So `workspace_floor` is a planner guard
  slab whose top sits 5 cm **below** the bench, not the bench itself.
- `ur5e_moveit_config/config/joint_limits.yaml:48` corroborates: *"only the
  flat workspace_floor slab, top z=-0.05"*.

`base1`/`base2` carry no recorded world pose in the registry (their `position`
entries are grasp/offset data at the origin, and the fixtures' live poses come
from Isaac TF), so there is no registry pose that contradicts this.

**Therefore `inject_intruder.py`'s usage example `position_xyz_world=(0.4,
-0.2, 0.76)` is not plausible as a bench-surface pose** — it is ~0.76 m in the
air, roughly half-way to the camera (which sits at z = 1.7,
`sim_moveit_bringup/config/cameras.yaml`). The same applies to
`spawn_random_intruders.py`'s `Z_SPAWN = 0.80` and `Z_STATIC = 0.785`, and to
this report's own Stage 3 §"geometry" note, which assumed a 0.76 table.

Why the live test still worked: the spawn script gives each object physics with
probability 0.7, and a physics object dropped from 0.80 simply falls to the
bench and settles — which is what you observed. A `physics=False` object at
`Z_STATIC = 0.785`, however, **hovers in mid-air** with nothing under it.

Recommended values (not applied — this is a report):

| Setting | Current | Should be |
|---|---|---|
| `inject_intruder.py` usage example z | 0.76 | ~0.03 (a 5 cm cube resting: half-height above 0) |
| `spawn_random_intruders.py` `Z_SPAWN` (physics, drops) | 0.80 | ~0.05–0.10 |
| `spawn_random_intruders.py` `Z_STATIC` (no physics) | 0.785 | `size_m / 2` above 0 |

Secondary consequence — **the camera standoff is larger than Stage 3 assumed**,
which makes small intruders even harder to see than that report said. With the
camera at z = 1.7 and the bench at 0, the standoff is 1.70 m (not 0.94 m).
From `focalLength 24.0 / horizontalAperture 20.955` the horizontal FOV is 47.2°,
giving a ~1.48 m-wide view:

| Object | % of frame width | ~% of frame area |
|---|---|---|
| 0.033 m (smallest spawned) | 2.2% | 0.05% |
| 0.05 m | 3.4% | 0.11% |
| 0.06 m (largest spawned) | 4.0% | 0.16% |
| 0.15 m (suggested plumbing-test size) | 10.1% | 1.0% |

So the Stage 3 conclusion stands and strengthens: an agent-chosen
`min_changed_frac` of 0.02 is ~20× too coarse for a 5 cm intruder, and even a
0.15 m object only reaches ~1% of frame area. A plumbing test that must fire
regardless of agent threshold choice wants a `min_changed_frac` well under
0.005, or an object larger than 0.15 m.

## 6. Summary of actions suggested (none taken)

1. Bring the cell up before the smoke run so the session pointer is rewritten
   (§1.4).
2. Fix the spawn heights in `inject_intruder.py`'s usage block and
   `spawn_random_intruders.py`'s `Z_SPAWN`/`Z_STATIC` (§5), and correct the
   Stage 3 report's camera-geometry note.
3. Before scoring any S2/S3 or injection run: teach the scorer `"authored"`
   coverage/provenance, make it ignore `purpose == "observation"` fires, and
   have it read observations + ground truth + the leak audit (§4).
4. Restate HANDOFF §6's known-failure numbers against the venv recipe (§3).
