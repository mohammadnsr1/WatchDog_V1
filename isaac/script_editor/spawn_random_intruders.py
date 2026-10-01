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
# workspace (parts sit around x 0.3-0.5, y -0.25-0.15, table top z ~0.76) --
# adjust to taste.
X_RANGE = (0.30, 0.55)
Y_RANGE = (-0.28, 0.18)
Z_SPAWN = 0.80          # spawn height; physics objects drop to the table from here
Z_STATIC = 0.785        # resting height used when an object has no physics

SIZE_RANGE_M = (0.025, 0.06)        # uniform per object
SHAPES = ("cube", "cylinder", "sphere")
PHYSICS_PROBABILITY = 0.7           # chance an object gets rigid-body physics
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
    physics = rng.random() < PHYSICS_PROBABILITY
    delay = rng.uniform(*DELAY_RANGE_S) if AT_STEP is None else 0
    ii.inject(
        f"intruder_{batch_tag}_{i}",
        shape=shape,
        size_m=round(rng.uniform(*SIZE_RANGE_M), 4),
        position_xyz_world=(
            round(rng.uniform(*X_RANGE), 4),
            round(rng.uniform(*Y_RANGE), 4),
            Z_SPAWN if physics else Z_STATIC,
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
