# S1 Class 1 injection — open in Isaac's Script Editor and run, once per run.
#
# The pinned call is copied from paper_results/batches/S1/batch.yaml `notes`
# (the authority). Do NOT edit the call mid-batch; if the position has to
# change, change batch.yaml first, before run 01.
#
# WHEN: after the bringup is up and $SD is verified, BEFORE the executor
# starts. The at_step trigger latches onto whatever ~/.watchdog/current_session
# names on its first successful read, so the check below refuses to queue
# unless that pointer names an existing, freshly created session dir.
#
# BETWEEN RUNS: do not call cancel_all() (it writes a removal record into the
# finished session). Reopen assembly_room.usd discarding changes, and never
# save the stage with the cube in it.

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
print(f"[s1_c1_inject] session dir: {_sd}  (modified {_age:.0f} s ago)")
if _age > MAX_SESSION_AGE_S:
    raise RuntimeError(
        f"{_sd} was last modified {_age/60:.1f} min ago -- stale pointer? "
        "Start the bringup for this run first."
    )

place_object(position_xyz_world=(0.08, -0.38, None),
             label="foreign cube on free bench area",
             expected_impact="benign",
             start="at_step=pick_u_orange")


