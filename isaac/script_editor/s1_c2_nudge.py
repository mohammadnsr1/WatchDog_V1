# S1 Class 2 nudge -- open in Isaac's Script Editor and run, once per run.
#
# Displaces a registered part by DELTA_XYZ at the first idle moment after
# STEP begins (the external-displacement fault). Ground truth goes to
# <session>/ground_truth_injections.jsonl (event_kind external_displacement,
# with pre/post poses). Also queues the benign Class 1 cube, so a nudge run
# keeps the same observation episode as every other run.
#
# Fill in the three values below BEFORE the run and copy them into the
# label's violation_steps ({step: STEP, object: OBJECT_ID}) and batch.yaml.
# Do not change them mid-batch for a run already made; a new combination is
# a new fault.
#
# WHEN: after the bringup is up and $SD is verified, BEFORE the executor
# starts (an at_step trigger queued after STEP has begun fires immediately).
#
# BETWEEN RUNS: reopen assembly_room.usd WITHOUT saving -- that also undoes
# the nudge. Never save the stage after a nudge.

OBJECT_ID = "hex_red"
DELTA_XYZ = (-0.05,0,0)
STEP = "pick_line_green" 

YAW_DEG = 0.0
PLACE_BENIGN_CUBE = True            # the pinned Class 1 cube (batch.yaml notes)

import sys, importlib, time
from pathlib import Path
sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1/isaac/script_editor")
importlib.invalidate_caches()
import inject_intruder, inject_scenarios
importlib.reload(inject_intruder); importlib.reload(inject_scenarios)
from inject_scenarios import *

MAX_SESSION_AGE_S = 15 * 60   # a pointer older than this is probably stale

_sd = Path(inject_intruder.resolve_session_dir())   # raises if missing
_age = time.time() - _sd.stat().st_mtime
print(f"[s1_c2_nudge] session dir: {_sd}  (modified {_age:.0f} s ago)")
if _age > MAX_SESSION_AGE_S:
    raise RuntimeError(
        f"{_sd} was last modified {_age/60:.1f} min ago -- stale pointer? "
        "Start the bringup for this run first."
    )

nudge_object(OBJECT_ID, DELTA_XYZ, yaw_deg=YAW_DEG,
             label=f"C2 nudge {OBJECT_ID} at {STEP}",
             expected_impact="violation",
             start=f"at_step={STEP}")

if PLACE_BENIGN_CUBE:
    place_object(position_xyz_world=(0.08, -0.38, None),
                 label="foreign cube on free bench area",
                 expected_impact="benign",
                 start="at_step=pick_u_orange")



