"""Spawn random intruder objects — paste this WHOLE file into Isaac's Script
Editor and Run.

Builds on isaac/script_editor/inject_intruder.py: every spawned object goes through
ii.inject(), so it lands under /World/WatchdogIntruders (no TF frame, not
agent-queryable), is name-checked against the registry, and is logged to the
live session's ground_truth_injections.jsonl at the moment it appears.

Edit the CONFIG block, Run. Run again for a fresh batch (names never repeat:
each batch carries a random tag). The seed is recorded in every ground-truth
line's `notes`, so a batch can be reproduced exactly by setting SEED to the
printed value.
"""

import random
import sys
import time

sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1/isaac/script_editor")
import inject_intruder as ii

# =========================== CONFIG ========================================
N_OBJECTS = 3

# Where objects may appear, in world/metres. Defaults bracket the FMB table
# workspace (parts sit around x 0.3-0.5, y -0.25-0.15; the bench top is world
# z ~0.01) -- adjust to taste.
X_RANGE = (0.30, 0.55)
Y_RANGE = (-0.28, 0.18)
# Spawn height is ALWAYS the resting default from inject_intruder
# (bench + half the object's height + 5 mm): the 2026-10-01 smoke batch
# spawned at z ~0.8 -- 0.79 m above the bench -- and a physics=False object
# hung in mid-air. There is no z knob here on purpose.

SIZE_RANGE_M = (0.025, 0.06)        # uniform per object
SHAPES = ("cube", "cylinder", "sphere")
# Every spawned object gets rigid-body physics. Set False ONLY deliberately
# (a no-physics object never settles; at the resting default it sits on the
# bench visually, but it will not react to contact).
PHYSICS = True
YAW_RANGE_DEG = (0.0, 360.0)

# Saturated random colors (one channel high, one low) so intruders stand out
# on camera instead of blending into the grey table.
def _random_color(rng):
    channels = [rng.uniform(0.7, 1.0), rng.uniform(0.0, 0.3), rng.uniform(0.0, 1.0)]
    rng.shuffle(channels)
    return tuple(channels)

EXPECTED_IMPACT = "benign"          # or "violates" if you drop one on a placement
LABEL_PREFIX = "random spawn"

# Timing: defaults = all appear immediately. DELAY_RANGE_S staggers each
# object by a random delay; AT_STEP instead makes the WHOLE batch appear on
# that step's begin_step. Use one or the other -- when AT_STEP is set the
# delays are ignored (at_step wins in inject_intruder.trigger_text).
AT_STEP = None                      # e.g. "place_u_green"
DELAY_RANGE_S = (0.0, 0.0)          # e.g. (5.0, 40.0)

SEED = None                         # int to reproduce a batch; None = random
# ===========================================================================

seed = SEED if SEED is not None else random.randrange(10**6)
rng = random.Random(seed)
batch_tag = f"{time.time_ns() % 0xFFFF:04x}"

print(f"[spawn_random_intruders] batch intruder_{batch_tag}_* "
      f"(seed={seed}, n={N_OBJECTS})")

for i in range(N_OBJECTS):
    shape = rng.choice(SHAPES)
    physics = PHYSICS
    delay = rng.uniform(*DELAY_RANGE_S) if AT_STEP is None else 0
    size = round(rng.uniform(*SIZE_RANGE_M), 4)
    ii.inject(
        f"intruder_{batch_tag}_{i}",
        shape=shape,
        size_m=size,
        position_xyz_world=(
            round(rng.uniform(*X_RANGE), 4),
            round(rng.uniform(*Y_RANGE), 4),
            None,   # resting default: bench + size/2 + 5 mm
        ),
        yaw_deg=round(rng.uniform(*YAW_RANGE_DEG), 1),
        color_rgb=tuple(round(c, 3) for c in _random_color(rng)),
        physics=physics,
        label=f"{LABEL_PREFIX} {batch_tag}/{i} ({shape})",
        expected_impact=EXPECTED_IMPACT,
        notes=f"spawn_random_intruders seed={seed} index={i}",
        at_step=AT_STEP,
        delay_s=delay,
    )

ii.list_injections()
