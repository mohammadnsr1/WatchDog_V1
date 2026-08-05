"""Rung 2 (primitive 1/4) — StationarityMonitor, disturbed by hand in Isaac Sim.

Validates one deterministic monitor primitive in isolation, halt_on_fire=False
(flag only — Rung 3 adds halt composition on top of this once each primitive
is independently trusted).

Contract sourcing for this test (read before running):
  There is no dedicated "stay stationary" contract/clause type, and none is
  needed. red_cube's OWN PickContract already contains a precondition clause
      object_position_error_m LE position_tol   (position_tol = 0.005 m here)
  authored for a different purpose (verifying red_cube's own pose before its
  own pick), but the physical content is generic: "position error from
  expected pose must stay within tolerance." This test reuses that exact
  clause as the provenance for a stationarity check on red_cube while GREEN
  is being manipulated. Chain stays intact: clause -> primitive -> parameter
  -> registry source; the clause's ORIGINAL role (precondition) does not have
  to match how it's being used here (bystander invariant during another
  object's step). That reuse judgment is the agent's job at arm time; a fixed
  clause-role table would miss it.

Because the disturbance is a manual drag in Isaac Sim, this cannot be fully
scripted. The harness:
  1. Arms the monitor and captures the reference pose.
  2. Phase A (quiet window): confirms NO fire while nothing touches the cube
     — a false-positive check against TF jitter alone.
  3. Phase B (prompted): asks you to drag red_cube now, polls for a fire.
  4. Confirms the monitor auto-disarmed after firing, and that the
     monitor_fire event looped back onto /watchdog/events into the
     recorder's own event buffer (same loopback check as Rung 1).

This bypasses the MCP tool layer on purpose, same as Rung 0/1 — it exercises
EvidenceRecorder + MonitorEngine + StationarityMonitor directly so a failure
points at the primitive, not the plumbing.

Run (full stack + Isaac Sim up, workspace sourced, from the test/ directory
so the sibling import below resolves):
    python3 watchdog_rung2_stationarity_test.py
    python3 watchdog_rung2_stationarity_test.py --quiet-sec 8 --prompt-sec 30
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy

from robot_backend.contracts.loader import load_pick_contract
from robot_backend.watchdog.monitors import StationarityMonitor
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.timebase import wall_ns

# Reuse the Tier-1 harness from the Rung 0/1 smoke test (sibling module in
# the same test/ directory).
from watchdog_smoke_test import WatchDogHarness

STEP_ID = "rung2_stationarity_test"
TARGET_OBJECT = "red_cube"
# grasp_width_m is instruction-sourced, not registry-sourced (see
# agents/tasks/single_step_pick.{executor,watchdog}.yaml's steps: block);
# this manual test script
# has no task YAML of its own, so it stands in with the same value directly.
TARGET_OBJECT_GRASP_WIDTH_M = 0.06
DEFAULT_QUIET_SEC = 6.0
DEFAULT_PROMPT_SEC = 25.0
POLL_SEC = 0.2


def _find_position_clause(contract, field: str):
    for clause in contract.preconditions + contract.effects + contract.invariants:
        if clause.field == field:
            return clause
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet-sec", type=float, default=DEFAULT_QUIET_SEC,
                     help="false-positive check window before prompting you to disturb the cube")
    ap.add_argument("--prompt-sec", type=float, default=DEFAULT_PROMPT_SEC,
                     help="how long to wait for a fire after prompting")
    ap.add_argument("--log-dir", default=str(Path.home() / ".watchdog"))
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    if not rclpy.ok():
        rclpy.init()

    harness = WatchDogHarness(log_dir)
    # This test only needs the engine for one monitor family; wire it in the
    # same way _ensure_watchdog does (recorder.engine set after construction).
    engine = MonitorEngine(event_bus=harness.event_bus, halt_client=None,
                           logger=harness.node.get_logger())
    harness.recorder.engine = engine

    ok = True
    try:
        print("waiting for recorder buffers to fill...")
        time.sleep(2.0)

        baseline = harness.recorder.latest_object_pose(TARGET_OBJECT)
        if baseline is None:
            print(f"FAIL: no buffered pose for {TARGET_OBJECT} yet.")
            return 1
        bp = baseline["pose"]["position"]
        print(f"{TARGET_OBJECT} baseline pose: "
              f"x={bp['x']:.4f} y={bp['y']:.4f} z={bp['z']:.4f}")

        # --- contract-derived parameter, per the docstring above ---
        step_spec = {
            "step_id": STEP_ID,
            "object_id": TARGET_OBJECT,
            "grasp_width_m": TARGET_OBJECT_GRASP_WIDTH_M,
        }
        contract = load_pick_contract(TARGET_OBJECT, step_spec)
        clause = _find_position_clause(contract, "object_position_error_m")
        if clause is None:
            print(f"FAIL: {TARGET_OBJECT}'s contract has no "
                  f"object_position_error_m clause to source epsilon_m from.")
            return 1
        epsilon_m = clause.threshold
        print(f"epsilon_m = {epsilon_m} m, sourced from {TARGET_OBJECT}'s own "
              f"contract clause: field={clause.field!r} "
              f"comparison={clause.comparison.value!r} threshold={clause.threshold} "
              f"(\"{clause.description}\")")

        clause_dict = clause.model_dump(mode="json")
        monitor = StationarityMonitor(
            params={"object_id": TARGET_OBJECT, "epsilon_m": epsilon_m},
            step_id=STEP_ID,
            recorder=harness.recorder,
            clause=clause_dict,
            halt_on_fire=False,
            monitor_id=None,
            armed_wall_ns=wall_ns(),
        )
        armed = engine.arm(monitor)
        print(f"armed monitor_id={armed['monitor_id']} "
              f"reference={armed['reference_values']}")

        # --- Phase A: quiet window, false-positive check ---
        print(f"\nPhase A: quiet window ({args.quiet_sec:.0f}s) — do NOT touch "
              f"{TARGET_OBJECT}. Checking for spurious fires from TF jitter alone.")
        t_end = time.monotonic() + args.quiet_sec
        spurious = []
        while time.monotonic() < t_end:
            spurious.extend(engine.pop_fires())
            time.sleep(POLL_SEC)
        if spurious:
            print(f"  FAIL: {len(spurious)} spurious fire(s) during the quiet "
                  f"window — epsilon_m={epsilon_m} may be too tight for TF "
                  f"jitter on this frame. First: {spurious[0]}")
            ok = False
        else:
            print("  PASS: no fires while undisturbed.")

        if not ok:
            print("\nStopping before Phase B: fix the false-positive issue "
                  "first, since a real disturbance test is meaningless on top "
                  "of a noisy baseline.")
            return 1

        # Re-arm for phase B (phase A already consumed the arming above, but
        # since nothing fired, the SAME monitor is still armed and unchanged —
        # no re-arm needed; the pops above only drained an empty queue).
        print(f"\nPhase B: go drag {TARGET_OBJECT} in Isaac Sim NOW. "
              f"Waiting up to {args.prompt_sec:.0f}s for a fire...")
        t_end = time.monotonic() + args.prompt_sec
        fires = []
        while time.monotonic() < t_end:
            fires.extend(engine.pop_fires())
            if fires:
                break
            time.sleep(POLL_SEC)

        if not fires:
            print(f"  FAIL: no fire within {args.prompt_sec:.0f}s. Either "
                  f"{TARGET_OBJECT} wasn't moved, or it was moved by less than "
                  f"epsilon_m={epsilon_m} m.")
            current = harness.recorder.latest_object_pose(TARGET_OBJECT)
            if current is not None:
                cp = current["pose"]["position"]
                import math
                moved = math.sqrt(
                    (cp["x"] - bp["x"]) ** 2 + (cp["y"] - bp["y"]) ** 2 + (cp["z"] - bp["z"]) ** 2
                )
                print(f"      current displacement from baseline: {moved:.4f} m "
                      f"(threshold {epsilon_m} m)")
            return 1

        fire = fires[0]
        print(f"  fire observed: triggering_value={fire['triggering_value']:.4f} m "
              f"(threshold {fire['threshold']} m, comparison {fire['comparison']!r})")

        # --- auto-disarm check ---
        snapshot = engine.snapshot()
        still_armed = any(m["monitor_id"] == armed["monitor_id"] for m in snapshot)
        print(f"  auto-disarm after fire: "
              f"{'FAIL (still armed)' if still_armed else 'PASS'}")
        ok &= not still_armed

        # --- loopback check: did monitor_fire make it onto /watchdog/events
        #     and back into the recorder's own event tail? ---
        time.sleep(0.3)  # let the pub/sub round-trip land
        window_start = fire["trigger_wall_ns"] - 1_000_000_000
        window_end = wall_ns()
        events = harness.recorder.get_events_window(window_start, window_end)
        fire_events = [e for e in events if e["event_type"] == "monitor_fire"]
        loopback_ok = len(fire_events) > 0
        print(f"  monitor_fire observed on /watchdog/events: "
              f"{'PASS' if loopback_ok else 'FAIL'} ({len(fire_events)} found)")
        ok &= loopback_ok

        if len(fires) > 1:
            print(f"  note: {len(fires) - 1} additional fire(s) queued "
                  f"(only possible if another monitor with the same id space "
                  f"fired too; harmless here since only one monitor was armed).")

        print(f"\nRUNG 2 / stationarity: {'PASS' if ok else 'FAIL'}")
        return 0 if ok else 1
    finally:
        harness.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\nremember to drag {TARGET_OBJECT} back to its original spot "
              f"before running other tests that assume its baseline pose.")


if __name__ == "__main__":
    sys.exit(main())
