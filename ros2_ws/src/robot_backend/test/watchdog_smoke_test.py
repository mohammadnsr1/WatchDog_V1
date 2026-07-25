"""Rung 0 + Rung 1 WatchDog smoke test — recorder liveness and halt authority.

This bypasses the MCP layer on purpose. The MCP tools are thin facades over
EvidenceRecorder / HaltClient / MonitorEngine; testing the machinery directly
isolates "does the WatchDog work" from "does the MCP plumbing work". Monitors
are NOT exercised here (that is Rung 2) — halt is triggered manually so the
halt path is validated on its own.

RUNG 0 — recorder alive, no robot:
  Sample the recorder's buffers over a few seconds and confirm joint states
  and per-object TF poses are accumulating at roughly their expected rates,
  and that TF staleness counters stay flat. If Rung 0 fails, nothing above it
  can be trusted: the recorder is the evidence substrate for everything.

RUNG 1 — halt path alone, no monitors:
  Start a long arm motion, halt it mid-flight, and check three things:
    1. does HaltClient.halt() return ok=True (MoveGroup cancel accepted),
    2. does it return PROMPTLY, or does it hit its 1 s timeout — the flagged
       risk is that halt() blocks a MultiThreadedExecutor worker waiting on
       the cancel-service response while other callbacks occupy the pool,
    3. did the arm ACTUALLY stop — verified from the recorder's own joint
       buffer, not from the return value, since a cancel can be accepted and
       still not propagate through the Isaac trajectory bridge.

Topology note: production runs the executor and the WatchDog as separate
processes. This harness runs both in ONE process as two separate nodes
(exec_node / watchdog_node), which is faithful enough for halt semantics —
the cancel still goes over the ROS graph — but is not a test of process
isolation.

Node ownership follows the established rule:
  - exec_node: owns the backend + the executor's EventBus (configure_bus).
    Spun only by the skills themselves during calls. The test never spins it.
    Publishers do not require spinning, so the bus works on an unspun node.
  - watchdog_node: owns the recorder, its own TF buffer, the WatchDog's
    EventBus, and the HaltClient. Spun continuously by a background
    MultiThreadedExecutor thread, exactly as _ensure_watchdog does.

Run (full stack up, workspace sourced):
    python3 watchdog_smoke_test.py --rung 0
    python3 watchdog_smoke_test.py --rung 1
    python3 watchdog_smoke_test.py --rung both
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_backend.events.bus import EventBus, configure_bus, shutdown_bus
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.utils.geometry import make_pose
from robot_backend.watchdog.halt import HaltClient
from robot_backend.watchdog.recorder import EvidenceRecorder, _load_registry
from robot_backend.watchdog.timebase import wall_ns

# ------------------------- CONFIG -------------------------
WORLD_FRAME = "world"
GREEN = "green_cube"
SAFE_POSE_NAME = "ready"
GRASP_WIDTH_M = 0.04
APPROACH_HEIGHT_M = 0.10
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

# Rung 0
RUNG0_SAMPLE_SEC = 5.0
# Expected rates. /joint_states is well above this in practice; the bar is set
# low because Rung 0 asks "is the stream alive", not "is it fast".
MIN_JOINT_RATE_HZ = 10.0
# The recorder's TF timer runs at TF_SAMPLE_HZ = 20.0; allow generous slack.
MIN_TF_RATE_HZ = 10.0

# Rung 1
HALT_DELAY_SEC = 2.0        # how long into the motion to fire the halt
HALT_LATENCY_BUDGET_SEC = 1.0   # halt() should return inside its own 1 s timeout
STOP_OBSERVE_SEC = 1.5      # how long to watch joints after halt
STOP_SETTLE_SEC = 0.4       # ignore this much right after halt (deceleration)
STOP_EPSILON_RAD = 0.01     # max joint travel that still counts as "stopped"
# Below this, we can't tell "already at target" apart from "never started" —
# criterion 3 (stopped) is only meaningful once this is confirmed exceeded.
PRE_HALT_MOTION_MIN_RAD = 0.02
PRE_HALT_WINDOW_SEC = 0.8   # window just before the halt, used to confirm motion
# Joints belonging to the gripper, excluded from the arm-stopped check.
GRIPPER_JOINT_MARKERS = ("finger", "knuckle")
# ----------------------------------------------------------


def _pose(x: float, y: float, z: float) -> Any:
    return make_pose(
        position={"x": x, "y": y, "z": z},
        orientation=dict(TOP_DOWN_QUAT),
        frame_id=WORLD_FRAME,
    )


def _arm_joints(positions: dict[str, float]) -> list[str]:
    """Arm joints only — the gripper moves independently of a halted trajectory."""
    return [
        name
        for name in positions
        if not any(marker in name.lower() for marker in GRIPPER_JOINT_MARKERS)
    ]


class WatchDogHarness:
    """Stand up the WatchDog Tier-1 machinery exactly as _ensure_watchdog does."""

    def __init__(self, log_dir: Path) -> None:
        self.node = Node("watchdog_smoke_test_node")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.registry = _load_registry()
        self.event_bus = EventBus(
            node=self.node,
            log_path=str(log_dir / "smoke_watchdog.jsonl"),
        )
        self.recorder = EvidenceRecorder(node=self.node, registry=self.registry)
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.halt_client = HaltClient(
            node=self.node,
            event_bus=self.event_bus,
            callback_group=self.client_cb_group,
        )
        # engine intentionally left unset: Rung 1 halts manually, no monitors.
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def shutdown(self) -> None:
        # Same teardown ordering as the MCP server / TFReader: stop the
        # executor, let the spin thread exit, then destroy the node.
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.event_bus.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# RUNG 0
# ---------------------------------------------------------------------------


def rung0(harness: WatchDogHarness) -> bool:
    print("\n===== RUNG 0: recorder liveness =====")
    rec = harness.recorder

    ids = rec.object_ids()
    print(f"  enabled registry objects being tracked: {ids or '(none)'}")
    if not ids:
        print("  FAIL: recorder resolved no object frames from the registry.")
        return False

    first = rec.counts()
    first_stale = rec.tf_counters()
    t0 = time.monotonic()
    time.sleep(RUNG0_SAMPLE_SEC)
    elapsed = time.monotonic() - t0
    last = rec.counts()
    last_stale = rec.tf_counters()

    ok = True

    joint_delta = last["joint_states"] - first["joint_states"]
    joint_rate = joint_delta / elapsed
    joint_ok = joint_rate >= MIN_JOINT_RATE_HZ
    ok &= joint_ok
    print(f"  joint_states : +{joint_delta} in {elapsed:.1f}s "
          f"({joint_rate:.1f} Hz) {'OK' if joint_ok else 'FAIL'}")

    for oid in ids:
        delta = last["object_poses"].get(oid, 0) - first["object_poses"].get(oid, 0)
        rate = delta / elapsed
        stale_delta = last_stale.get(oid, 0) - first_stale.get(oid, 0)
        pose_ok = rate >= MIN_TF_RATE_HZ and stale_delta == 0
        ok &= pose_ok
        print(f"  pose[{oid}] : +{delta} ({rate:.1f} Hz), "
              f"staleness +{stale_delta} {'OK' if pose_ok else 'FAIL'}")
        if stale_delta > 0:
            print(f"      -> TF lookups are failing for {oid}. Check that its "
                  f"tf_frame/lookup_frame in model_registry.yaml match the "
                  f"frames actually published.")

    snap = rec.latest_joint_state()
    if snap is None:
        print("  latest_joint_state: None  FAIL")
        ok = False
    else:
        arm = _arm_joints(snap["positions"])
        print(f"  latest_joint_state: {len(snap['positions'])} joints "
              f"({len(arm)} arm), wall_ns={snap['wall_ns']}")

    events = rec.latest_events(5)
    print(f"  events buffered: {last['events']} (last {len(events)} shown)")
    for ev in events:
        print(f"      {ev['event_type']:<16} {ev['envelope']['wall_time_iso']}")
    if last["events"] == 0:
        print("      note: no events yet is expected if nothing has run.")

    print(f"  RUNG 0: {'PASS' if ok else 'FAIL'}")
    return ok


# ---------------------------------------------------------------------------
# RUNG 1
# ---------------------------------------------------------------------------


def _max_joint_travel(rec: EvidenceRecorder, start: int, end: int) -> tuple[float, str, int]:
    """Largest absolute arm-joint displacement inside a wall-ns window."""
    rows = rec.get_joint_window(start, end, None, 500)
    if len(rows) < 2:
        return 0.0, "(insufficient samples)", len(rows)
    names = _arm_joints(rows[0]["positions"])
    worst = 0.0
    worst_name = "(no joint moved)"
    for name in names:
        values = [r["positions"][name] for r in rows if name in r["positions"]]
        if len(values) < 2:
            continue
        travel = max(values) - min(values)
        if travel > worst:
            worst = travel
            worst_name = name
    return worst, worst_name, len(rows)


def rung1(harness: WatchDogHarness, backend: Any) -> bool:
    print("\n===== RUNG 1: halt path (no monitors) =====")
    rec = harness.recorder

    target = rec.latest_object_pose(GREEN)
    if target is None:
        print(f"  FAIL: no buffered pose for {GREEN}; run rung 0 first.")
        return False
    p = target["pose"]["position"]

    print("  moving to safe pose (setup)...")
    move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
    time.sleep(1.0)

    motion: dict[str, Any] = {}

    def _run_motion() -> None:
        started = time.monotonic()
        try:
            move_grasp_center_to_pose_skill(
                backend,
                _pose(p["x"], p["y"], p["z"] + APPROACH_HEIGHT_M),
                gripper_width_m=GRASP_WIDTH_M,
            )
            motion["ok"] = True
        except Exception as exc:  # noqa: BLE001
            # A cancelled goal surfacing as an exception here is a SUCCESS
            # signal for this test, not a failure — it means the executor
            # noticed the halt.
            motion["ok"] = False
            motion["error"] = f"{type(exc).__name__}: {exc}"
        motion["duration_sec"] = time.monotonic() - started

    print(f"  starting motion to ({p['x']:.3f}, {p['y']:.3f}, "
          f"{p['z'] + APPROACH_HEIGHT_M:.3f}) ...")
    thread = threading.Thread(target=_run_motion, daemon=True)
    thread.start()
    time.sleep(HALT_DELAY_SEC)

    if not thread.is_alive():
        print(f"  INCONCLUSIVE: motion finished in "
              f"{motion.get('duration_sec', 0):.2f}s, before the halt fired.")
        print(f"      -> lower HALT_DELAY_SEC (currently {HALT_DELAY_SEC}s) or "
              f"pick a farther target, then rerun.")
        return False

    # Confirm the arm is ACTUALLY moving before we halt it. Without this, a
    # halt fired before real motion starts (or after it's already arrived)
    # would make "stopped after halt" true but meaningless — travel would
    # read ~0 either way, for an unrelated reason.
    pre_halt_end = wall_ns()
    pre_halt_start = pre_halt_end - int(PRE_HALT_WINDOW_SEC * 1_000_000_000)
    pre_travel, pre_joint, pre_samples = _max_joint_travel(rec, pre_halt_start, pre_halt_end)
    was_moving = pre_travel >= PRE_HALT_MOTION_MIN_RAD
    print(f"  pre-halt motion check: {pre_travel:.4f} rad on {pre_joint} over "
          f"{PRE_HALT_WINDOW_SEC:.1f}s ({pre_samples} samples) "
          f"{'OK, arm is moving' if was_moving else 'TOO LOW'}")
    if not was_moving:
        print(f"      -> arm shows <{PRE_HALT_MOTION_MIN_RAD} rad movement right "
              f"before the halt fires. Either the trajectory hasn't started "
              f"yet or it already arrived. Increase HALT_DELAY_SEC (currently "
              f"{HALT_DELAY_SEC}s) or move to a farther target, then rerun — "
              f"criterion 3 below is not trustworthy until this passes.")

    print(f"  firing halt {HALT_DELAY_SEC:.1f}s into the motion...")
    t_call = time.monotonic()
    halt_wall = wall_ns()
    result = harness.halt_client.halt("rung1_smoke", "manual_test", "rung 1 halt smoke test")
    latency = time.monotonic() - t_call

    print(f"  halt returned ok={result.get('ok')} in {latency * 1000:.0f} ms")
    print(f"      detail: {result.get('detail')}")

    # Let the arm decelerate, then watch whether it is still moving.
    time.sleep(STOP_SETTLE_SEC)
    observe_start = wall_ns()
    time.sleep(STOP_OBSERVE_SEC)
    observe_end = wall_ns()

    travel, joint, samples = _max_joint_travel(rec, observe_start, observe_end)
    stopped = travel <= STOP_EPSILON_RAD

    thread.join(timeout=10.0)

    # --- criteria ---
    halt_ok = bool(result.get("ok"))
    latency_ok = latency <= HALT_LATENCY_BUDGET_SEC

    print(f"\n  [1] cancel accepted    : {'PASS' if halt_ok else 'FAIL'}")
    print(f"  [2] returned promptly  : {'PASS' if latency_ok else 'FAIL'} "
          f"({latency * 1000:.0f} ms, budget {HALT_LATENCY_BUDGET_SEC * 1000:.0f} ms)")
    stop_check_valid = was_moving
    print(f"  [3] arm actually stopped: "
          f"{'PASS' if (stopped and stop_check_valid) else ('FAIL' if stop_check_valid else 'INVALID')} "
          f"(max travel {travel:.4f} rad on {joint} over "
          f"{STOP_OBSERVE_SEC:.1f}s, {samples} samples, "
          f"epsilon {STOP_EPSILON_RAD})")
    if not stop_check_valid:
        print("      -> INVALID, not FAIL: the pre-halt motion check above did "
              "not confirm the arm was moving, so a low post-halt travel number "
              "doesn't tell us anything. Fix that check first, then this one "
              "becomes meaningful.")
    print(f"  motion thread: ok={motion.get('ok')} "
          f"duration={motion.get('duration_sec', 0):.2f}s "
          f"{motion.get('error', '')}")

    # Did the halt event make it onto the shared topic and back into the
    # recorder's own event buffer? (Exercises the loopback the WatchDog relies
    # on for its unified timeline.)
    halt_events = [
        e for e in rec.get_events_window(halt_wall - 1_000_000_000, wall_ns())
        if e["event_type"] == "halt"
    ]
    loopback_ok = len(halt_events) > 0
    print(f"  [4] halt event observed on /watchdog/events: "
          f"{'PASS' if loopback_ok else 'FAIL'} ({len(halt_events)} found)")

    ok = halt_ok and latency_ok and stopped and stop_check_valid and loopback_ok

    if not stopped:
        print("\n      -> Cancel was accepted but the arm kept moving: the "
              "cancel is not propagating through to trajectory execution "
              "(check the Isaac trajectory bridge). halt_on_fire cannot be "
              "trusted until this passes.")
    if not latency_ok:
        print("\n      -> halt() is slow to return. If detail says 'timed out', "
              "the cancel-service response is not being processed — likely "
              "executor worker-thread starvation. Consider a dedicated "
              "executor/callback group for the halt client.")

    print(f"\n  RUNG 1: {'PASS' if ok else 'FAIL'}")
    return ok


# ---------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rung", choices=["0", "1", "both"], default="both")
    ap.add_argument("--log-dir", default=str(Path.home() / ".watchdog"))
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    if not rclpy.ok():
        rclpy.init()

    harness = WatchDogHarness(log_dir)

    # Executor side: one node owning the backend AND the executor's event bus,
    # mirroring the MCP servers. Skills spin this node themselves; the test
    # never does. Publishing needs no spin.
    exec_node = Node("watchdog_smoke_test_exec")
    configure_bus(node=exec_node, log_path=str(log_dir / "smoke_exec.jsonl"))
    backend = create_ros_moveit_backend(node=exec_node)

    print("waiting for recorder buffers to fill...")
    time.sleep(2.0)

    results: dict[str, bool] = {}
    try:
        if args.rung in ("0", "both"):
            results["rung0"] = rung0(harness)
        if args.rung in ("1", "both"):
            if args.rung == "both" and not results.get("rung0", False):
                print("\n===== RUNG 1: SKIPPED (rung 0 failed) =====")
                print("  Rung 1 verifies the arm stopped using the recorder's "
                      "joint buffer. If the recorder is not healthy, rung 1 "
                      "cannot produce a trustworthy answer.")
                results["rung1"] = False
            else:
                results["rung1"] = rung1(harness, backend)
    finally:
        print("\n" + "=" * 55)
        for name, ok in results.items():
            print(f"  {name}: {'PASS' if ok else 'FAIL'}")
        print(f"  logs: {log_dir}/smoke_exec.jsonl, {log_dir}/smoke_watchdog.jsonl")

        shutdown_bus()
        harness.shutdown()
        try:
            exec_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
