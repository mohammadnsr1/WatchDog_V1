#!/usr/bin/env python3
"""Physics tuning for the assembly cell: stop the grasped part from vibrating.

Run inside Isaac Sim (Script Editor, or via the kit-mcp bridge's run_python).
Applying it to a loaded stage authors into the stage's edit target; save the
stage afterwards to make it stick.

THE PROBLEM
-----------
Parts buzzed in the gripper badly enough to wreck placement: measured live,
hex_blue reached 60 rad/s of angular velocity and hex_red reversed spin
direction 105 times in 295 physics frames. Near-zero linear velocity with
large oscillating angular velocity is contact chatter, not motion.

THE CAUSE
---------
finger_joint's drive was far too stiff for the solver rate:

    JointEquivalentInertia = 0.00506
    drive stiffness        = 1600 per DEGREE  (USD angular drives use degrees)
                           ~ 91,700 N*m/rad
    => natural frequency   ~ 680 Hz
    physics rate           = 60 Hz

A 60 Hz solver cannot integrate a ~680 Hz oscillator; the drive overshoots,
the contact pushes back, and it rings. maxForce was `inf` and
maxJointVelocity was 5729 deg/s (100 rad/s), so nothing bounded the
overshoot either.

WHAT WAS TRIED, AND WHAT ACTUALLY HAPPENED
------------------------------------------
Raising timeStepsPerSecond 60 -> 240 ELIMINATES the chatter completely:

    part               max angular rad/s      wz sign flips
    hex_blue           60.1  -> 0.05           21  -> 0
    u_green            25.9  -> 0.33           37  -> 0
    inverted_u_brown   16.5  -> 0.00           65  -> 0
    hex_red             0.20 -> 0.08          105  -> 0

and it costs nothing: 240 Hz measured 1.7x realtime, FASTER than 60 Hz
managed under load, because the solver stops burning iterations fighting
itself.

BUT 240 Hz BREAKS THE GRIPPER. Verified with a real grasp of hex_blue: the
guarded move stopped at 103 mm (barely closed) with stop_reason
"effort_plateau" at effort 0.04 against a force_threshold of 15.

    rg2_controller.py stops on effort_plateau after only
    effort_plateau_ticks = 2 consecutive samples with |d_effort| < 0.05.

The controller polls at 50 Hz and reads effort from /joint_states, which the
ROS_JointStates OmniGraph publishes per RENDERED frame, not per physics step.
Raising the physics rate changes how many fresh effort samples arrive per unit
of simulated closing motion, so two flat early samples now look like a
plateau. Reverting to 60 Hz makes the grasp succeed again (30.1 mm, effort
17.3, stop_reason "force_threshold") -- and brings the chatter back
(hex_red: 398 sign flips in 2325 frames, ~10 Hz).

So the rate is BOTH the fix for the chatter and the cause of the grasp
failure. Resolving it needs one of:

  (a) 240 Hz plus a rate-independent plateau test in rg2_controller (raise
      effort_plateau_ticks, or gate on elapsed time rather than tick count).
      Touches grasp-detection logic, which is contract-relevant -- the effort
      clauses and max_clamp_effort are judged against it.
  (b) 240 Hz plus making ROS_JointStates tick on physics steps so the effort
      sample rate scales with the solver.
  (c) An intermediate rate (120 Hz) -- may damp enough while leaving the
      controller enough samples. Untested.

WHAT THIS SCRIPT APPLIES BY DEFAULT
-----------------------------------
The conservative subset that is verified not to break the grasp:

  * solverVelocityIterationCount 1 -> 4. Position iterations (already 32,
    evidently raised while fighting this) fix penetration; VELOCITY
    iterations are what damp residual spin, which is what this is.
  * contactOffset 0.002 -> 0.005 on the parts and the fixture. At 2 mm a fast
    finger can penetrate before a contact exists, so the first contact
    resolved is already deep. restOffset stays 0.

These two ALONE do not eliminate the chatter -- only the rate does. Set
TIME_STEPS_PER_SECOND below and pass tune_rate=True once the controller
question above is settled.

The finger drive is left at its original 1600/60/inf/5729. Detuning it to
400/40/400/200 was tried and made the grasp fail in a second, independent way
(too little authority to clamp), so it is not part of the fix.

Friction (1.0/1.0) and restitution (0.0) were already fine and are untouched.
"""

from __future__ import annotations

PHYSICS_SCENE = "/physicsScene"
ARTICULATION_ROOT = "/World/ur5e/root_joint"
FINGER_JOINT = "/World/ur5e/Rg2_mn/onrobot_rg2/joints/finger_joint"
PARTS = ["u_green", "inverted_u_brown", "hex_blue", "hex_red", "base2"]

# Only applied when apply(tune_rate=True). Fixes the chatter; breaks the
# gripper's plateau detection until rg2_controller is made rate-independent.
# See the module docstring.
TIME_STEPS_PER_SECOND = 240

SOLVER_VELOCITY_ITERATIONS = 4
CONTACT_OFFSET_M = 0.005
REST_OFFSET_M = 0.0

# --- Mimic-joint stiffness: the lever that actually sets clamp force --------
#
# Only finger_joint is DRIVEN. The other five have no drive and carry
# PhysxMimicJointAPI:rotX referencing finger_joint (gearing +1 on the three
# knuckles, -1 on the two inner fingers). Their naturalFrequency sets how much
# commanded travel survives to the part instead of being absorbed as
# constraint yield.
#
# Measured 2026-08-31 on u_green at 3 mm clearance, 240 Hz, via
# test/rg2_gripper_stiffness_sweep.py. Settled clamp effort / grip width error
# in mm, at dampingRatio 0.8 and 2.0:
#
#   naturalFrequency   dr=0.8            dr=2.0
#   1000 (authored)    20.2 (+0.30)      17.5 (+0.34)
#   1500               33.7 (+0.26)      26.9 (+0.20)
#   2000               49.1 (+0.17)      40.0 (+0.07)
#   3000               82.6 (-0.08)      62.8 (-0.05)
#   5000              144.2 (+0.21)      98.4 (-0.14)
#
# A NEGATIVE width error means the fingers are penetrating a rigid part --
# over-clamping. That starts at 3000, so 3000+ is out regardless of how firm
# it looks. 2000/2.0 gives 2.2x the authored clamp with +0.07 mm width error,
# the closest to zero of the 18 cells swept.
#
# At the authored 1000 the commanded 3 mm clearance never reached the part:
# right_outer_knuckle_joint yielded ~2.6 mm under load, leaving ~29 um of
# actual contact penetration, which is why grasps read effort ~20 yet slipped.
#
# The finger_joint DRIVE is deliberately not tuned here: stiffness 400->6400
# (16x) and damping 60->240 (4x) moved settled clamp only 16.7->20.9. The
# drive sits upstream of the mimic constraint, so stiffening it changes
# nothing. (The docstring's note that detuning the drive to 400/40/400/200
# failed with "too little authority to clamp" is therefore misattributed --
# drive stiffness barely affects clamp at all.)
#
# Changing these invalidates every per-object spec.grasp.force_threshold_n in
# model_registry.yaml, which was measured against the soft constraint.
# Re-measure per part before scoring.
MIMIC_JOINTS = [
    "/World/ur5e/Rg2_mn/onrobot_rg2/joints/left_inner_knuckle_joint",
    "/World/ur5e/Rg2_mn/onrobot_rg2/joints/right_inner_knuckle_joint",
    "/World/ur5e/Rg2_mn/onrobot_rg2/joints/right_outer_knuckle_joint",
    "/World/ur5e/Rg2_mn/onrobot_rg2/joints/left_inner_finger_joint",
    "/World/ur5e/Rg2_mn/onrobot_rg2/joints/right_inner_finger_joint",
]
MIMIC_NATURAL_FREQUENCY = 2000.0   # authored value was 1000.0
MIMIC_DAMPING_RATIO = 2.0          # authored value was 0.8


def apply(verbose: bool = True, tune_rate: bool = False,
          tune_mimic: bool = True) -> dict:
    """Apply the tuning to the currently loaded stage. Returns before/after.

    tune_rate=False (default) applies only the changes verified not to break
    the grasp. tune_rate=True additionally raises timeStepsPerSecond. NOTE:
    the historical blocker for that -- the guarded move stopping early with
    "effort_plateau" -- is gone: rg2_controller's plateau branch is now
    unreachable at its default effort_plateau_gate_frac of 1.0. Measured at
    480 Hz it tightens settled-clamp spread to 6% (from 17%) but does not
    change the mimic yield, so it is a repeatability knob, not a grip fix.

    tune_mimic=True (default) sets the five mimic joints' naturalFrequency /
    dampingRatio -- the parameter that actually sets clamp force. See the
    MIMIC_* constants for the sweep this came from. Pass False to leave the
    authored 1000 / 0.8 in place.
    """
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    before: dict = {}
    after: dict = {}

    def _set(prim_path: str, attr: str, value, key: str):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            before[key] = after[key] = f"<missing prim {prim_path}>"
            return
        a = prim.GetAttribute(attr)
        if not a:
            before[key] = after[key] = f"<missing attr {attr}>"
            return
        before[key] = str(a.Get())
        a.Set(value)
        after[key] = str(a.Get())

    if tune_rate:
        _set(PHYSICS_SCENE, "physxScene:timeStepsPerSecond",
             TIME_STEPS_PER_SECOND, "timeStepsPerSecond")
    _set(ARTICULATION_ROOT, "physxArticulation:solverVelocityIterationCount",
         SOLVER_VELOCITY_ITERATIONS, "solverVelocityIterationCount")

    if tune_mimic:
        for path in MIMIC_JOINTS:
            name = path.rsplit("/", 1)[-1]
            _set(path, "physxMimicJoint:rotX:naturalFrequency",
                 MIMIC_NATURAL_FREQUENCY, f"{name}:naturalFrequency")
            _set(path, "physxMimicJoint:rotX:dampingRatio",
                 MIMIC_DAMPING_RATIO, f"{name}:dampingRatio")

    offsets_before, offsets_after = {}, {}
    for name in PARTS:
        root = stage.GetPrimAtPath(f"/World/{name}")
        if not root:
            continue
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            ca = prim.GetAttribute("physxCollision:contactOffset")
            ra = prim.GetAttribute("physxCollision:restOffset")
            if ca:
                offsets_before[name] = round(float(ca.Get() or 0.0), 5)
                ca.Set(CONTACT_OFFSET_M)
                offsets_after[name] = round(float(ca.Get()), 5)
            if ra:
                ra.Set(REST_OFFSET_M)
    before["contact_offsets"] = offsets_before
    after["contact_offsets"] = offsets_after

    if verbose:
        for k in before:
            print(f"  {k}: {before[k]} -> {after[k]}")
    return {"before": before, "after": after}


def save_stage() -> str:
    """Persist the tuning into the stage's root layer file on disk."""
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    layer = stage.GetRootLayer()
    layer.Save()
    return layer.identifier


def verify(duration_note: str = "let this run, then call read_stats()") -> str:
    """Install a physics-step recorder measuring per-part vibration.

    Sign flips of angular velocity about z are the discriminator: a part being
    carried rotates smoothly and flips rarely, a chattering part reverses many
    times per second. Call read_stats() afterwards, then remove().
    """
    import omni.physx
    import numpy as np
    from isaacsim.core.prims import RigidPrim

    prims, stats = {}, {}
    for name in PARTS[:4]:
        rp = RigidPrim(f"/World/{name}")
        rp.initialize()
        prims[name] = rp
        stats[name] = {"n": 0, "max_ang": 0.0, "max_lin": 0.0,
                       "moving": 0, "flips": 0, "last": 0.0}

    def _cb(dt):
        for name, rp in prims.items():
            try:
                v = rp.get_velocities()[0]
            except Exception:
                continue
            lin = float(np.linalg.norm(v[:3]))
            ang = float(np.linalg.norm(v[3:]))
            s = stats[name]
            s["n"] += 1
            s["max_ang"] = max(s["max_ang"], ang)
            s["max_lin"] = max(s["max_lin"], lin)
            if ang > 0.05:
                s["moving"] += 1
            wz = float(v[5])
            if s["last"] * wz < 0 and abs(wz) > 0.05:
                s["flips"] += 1
            s["last"] = wz

    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_cb)
    globals()["vibsub"] = sub          # plain names: persistent sessions drop
    globals()["vibstats"] = stats      # underscore-prefixed globals
    return duration_note


def read_stats() -> dict:
    stats = globals().get("vibstats", {})
    return {
        k: {"frames": v["n"],
            "max_ang_rad_s": round(v["max_ang"], 3),
            "max_lin_m_s": round(v["max_lin"], 4),
            "pct_rotating": round(100.0 * v["moving"] / v["n"], 1) if v["n"] else 0,
            "wz_sign_flips": v["flips"]}
        for k, v in stats.items()
    }


def remove() -> None:
    """Detach the recorder -- it reads tensors every physics step."""
    sub = globals().pop("vibsub", None)
    if sub is not None:
        try:
            sub.unsubscribe()
        except Exception:
            pass


if __name__ == "__main__":
    apply()
