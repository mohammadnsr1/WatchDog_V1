#!/usr/bin/env python3
"""Sweep the RG2's stiffness/damping parameters and measure the resulting clamp.

WHY THIS EXISTS
---------------
Measured 2026-08-31: at the authored parameters the commanded grasp clearance
never reaches the part. The probe commands the finger toward
angle_for_width(grasp_width - GRASP_CLEARANCE_M), but the mimic constraint on
right_outer_knuckle_joint yields ~2.6 mm under load, so actual contact
penetration is ~29 um and settled clamp effort sits near 15. Raising the mimic
constraint's naturalFrequency from 1000 to 10000 raised settled clamp to ~220
and closed the fingers 1.1 mm further -- the clearance finally reaching the
part. 10000 is far too firm; the usable value is somewhere between.

There are TWO independent stiffness/damping pairs on this gripper, and they do
different things:

  * finger_joint drive  -- stiffness / damping (USD angular drive, per DEGREE).
    The only DRIVEN joint. Sets how hard the master pushes for a given
    tracking error.
  * the five mimic joints -- naturalFrequency / dampingRatio
    (PhysxMimicJointAPI:rotX). No drive; these enforce the linkage relation.
    Their stiffness sets how much commanded travel survives to the part
    instead of being absorbed by constraint yield. Measured to dominate.

WHAT THIS SCRIPT DOES
---------------------
Parks the arm at the grasp pose ONCE (only the gripper cycles afterwards, so a
cell is ~15 s rather than the ~50 s a full pick cycle costs), then per cell:
sets the physics parameters in the live stage through the omni.kit.mcp bridge,
opens, runs a torque-mode guarded_move, holds, and records the settled clamp.

Physics parameters are ALWAYS restored in a finally block, including on
KeyboardInterrupt, and the stage is never saved -- an Isaac restart would
discard them anyway, but a half-swept stage left behind would silently
invalidate every later run.

Run (full stack up, workspace sourced, from test/):
    python3 rg2_gripper_stiffness_sweep.py --grid mimic
    python3 rg2_gripper_stiffness_sweep.py --grid drive
    python3 rg2_gripper_stiffness_sweep.py --grid mimic \
        --natfreq 1000,2000,4000 --dampratio 0.8,2.0
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_backend.skills.gripper_skills import (
    GRASP_CLEARANCE_M,
    _call_guarded_move,
    _resolve_guarded_move_action_name,
)
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.utils.geometry import compose_poses, make_pose

# Sibling modules in test/ -- reuse rather than reimplement.
from rg2_effort_sweep import (  # noqa: E402
    TFReader,
    _load_grasp_center_offset,
    _resolve_tf_frame,
    KNOWN_OBJECT_WIDTHS_M,
    WORLD_FRAME,
    SAFE_POSE_NAME,
    APPROACH_HEIGHT_M,
    TOP_DOWN_QUAT,
)

KIT_PY = str(Path.home() / ".local/share/uv/tools/kit-mcp/bin/python")
KIT_PORT = "9009"
JOINT_BASE = "/World/ur5e/Rg2_mn/onrobot_rg2/joints/"
FINGER_JOINT = "finger_joint"
MIMIC_JOINTS = [
    "left_inner_knuckle_joint",
    "right_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "left_inner_finger_joint",
    "right_inner_finger_joint",
]
GRIPPER_JOINTS = [FINGER_JOINT] + MIMIC_JOINTS
MULT = [1.0, -1.0, -1.0, -1.0, 1.0, 1.0]

SETTLE_SEC = 1.0
HOLD_SEC = 2.0
POST_OPEN_SEC = 0.8


# ---------------------------------------------------------------------------
# Kit bridge
# ---------------------------------------------------------------------------

def kit(code: str, timeout: float = 60.0):
    """Run Python inside the live Isaac Sim via the omni.kit.mcp bridge."""
    proc = subprocess.run(
        [KIT_PY, "-m", "kit_mcp.client", "run_python",
         json.dumps({"code": code}), "--port", KIT_PORT],
        capture_output=True, text=True, timeout=timeout,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"kit bridge failed ({proc.returncode}): "
                           f"{proc.stderr.strip() or proc.stdout.strip()}")
    return json.loads(proc.stdout).get("result")


_READ_PARAMS = f"""
import omni.usd
stage = omni.usd.get_context().get_stage()
base = "{JOINT_BASE}"
out = {{}}
fj = stage.GetPrimAtPath(base + "{FINGER_JOINT}")
out["drive_stiffness"] = fj.GetAttribute("drive:angular:physics:stiffness").Get()
out["drive_damping"] = fj.GetAttribute("drive:angular:physics:damping").Get()
out["mimic"] = {{}}
for j in {MIMIC_JOINTS!r}:
    p = stage.GetPrimAtPath(base + j)
    out["mimic"][j] = [
        p.GetAttribute("physxMimicJoint:rotX:naturalFrequency").Get(),
        p.GetAttribute("physxMimicJoint:rotX:dampingRatio").Get(),
    ]
out["timeStepsPerSecond"] = stage.GetPrimAtPath(
    "/physicsScene").GetAttribute("physxScene:timeStepsPerSecond").Get()
result = out
"""


def read_params() -> dict:
    return kit(_READ_PARAMS)


def set_drive(stiffness=None, damping=None) -> None:
    sets = []
    if stiffness is not None:
        sets.append(f'fj.GetAttribute("drive:angular:physics:stiffness").Set({float(stiffness)})')
    if damping is not None:
        sets.append(f'fj.GetAttribute("drive:angular:physics:damping").Set({float(damping)})')
    if not sets:
        return
    kit("import omni.usd\n"
        "stage = omni.usd.get_context().get_stage()\n"
        f'fj = stage.GetPrimAtPath("{JOINT_BASE}{FINGER_JOINT}")\n'
        + "\n".join(sets) + "\nresult = True\n")


def set_mimic(natfreq=None, dampratio=None) -> None:
    sets = []
    if natfreq is not None:
        sets.append(f'p.GetAttribute("physxMimicJoint:rotX:naturalFrequency").Set({float(natfreq)})')
    if dampratio is not None:
        sets.append(f'p.GetAttribute("physxMimicJoint:rotX:dampingRatio").Set({float(dampratio)})')
    if not sets:
        return
    body = "\n        ".join(sets)
    kit("import omni.usd\n"
        "stage = omni.usd.get_context().get_stage()\n"
        f"for j in {MIMIC_JOINTS!r}:\n"
        f'    p = stage.GetPrimAtPath("{JOINT_BASE}" + j)\n'
        f"    if p and p.IsValid():\n"
        f"        {body}\n"
        "result = True\n")


def restore(saved: dict) -> None:
    set_drive(saved["drive_stiffness"], saved["drive_damping"])
    first = next(iter(saved["mimic"].values()))
    set_mimic(first[0], first[1])


# ---------------------------------------------------------------------------
# Joint-state reader (all six gripper joints, not just finger_joint)
# ---------------------------------------------------------------------------

class GripperState(Node):
    def __init__(self):
        super().__init__("rg2_stiffness_sweep_state")
        self._lock = threading.Lock()
        self._pos: dict[str, float] = {}
        self._eff: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._cb, 50)

    def _cb(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        pos, eff = {}, {}
        for j in GRIPPER_JOINTS:
            i = idx.get(j)
            if i is None:
                continue
            if i < len(msg.position):
                pos[j] = msg.position[i]
            if msg.effort and i < len(msg.effort):
                eff[j] = msg.effort[i]
        if pos:
            with self._lock:
                self._pos, self._eff = pos, eff

    def snapshot(self) -> tuple[dict, dict]:
        with self._lock:
            return dict(self._pos), dict(self._eff)

    def wait_ready(self, timeout: float = 10.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if FINGER_JOINT in self._pos:
                    return True
            time.sleep(0.05)
        return False


def spin_bg(node: Node) -> threading.Thread:
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    return t


# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="u_green")
    ap.add_argument("--width_m", type=float, default=None)
    ap.add_argument("--grid", choices=["mimic", "drive"], default="mimic",
                    help="mimic = naturalFrequency x dampingRatio on the five "
                         "passive joints; drive = stiffness x damping on "
                         "finger_joint")
    ap.add_argument("--natfreq", default="1000,2000,3000,5000")
    ap.add_argument("--dampratio", default="0.8,2.0")
    ap.add_argument("--stiffness", default="400,1600,6400")
    ap.add_argument("--damping", default="60,240")
    ap.add_argument("--force_threshold", type=float, default=16.0)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    oid = args.object
    width = args.width_m if args.width_m is not None else KNOWN_OBJECT_WIDTHS_M.get(oid)
    if width is None:
        print(f"ERROR: pass --width_m for {oid!r}")
        return 1
    target_width = max(0.0, width - GRASP_CLEARANCE_M)
    csv_path = args.csv or f"rg2_stiffness_sweep_{args.grid}.csv"

    if args.grid == "mimic":
        cells = [("natfreq", float(a), "dampratio", float(b))
                 for a in args.natfreq.split(",")
                 for b in args.dampratio.split(",")]
    else:
        cells = [("stiffness", float(a), "damping", float(b))
                 for a in args.stiffness.split(",")
                 for b in args.damping.split(",")]

    saved = read_params()
    print(f"authored params: drive stiffness={saved['drive_stiffness']} "
          f"damping={saved['drive_damping']}; "
          f"mimic nf/dr={next(iter(saved['mimic'].values()))}; "
          f"physics={saved['timeStepsPerSecond']} Hz")
    print(f"object={oid} width={width} clearance={GRASP_CLEARANCE_M} "
          f"-> probe floor {target_width:.4f} m; force_threshold={args.force_threshold}")
    print(f"{len(cells)} cells\n")

    if not rclpy.ok():
        rclpy.init()

    tf_frame = _resolve_tf_frame(oid)
    tf = TFReader(WORLD_FRAME)
    if not tf.wait_ready(tf_frame):
        print(f"ERROR: no TF for {tf_frame}")
        return 1

    state = GripperState()
    spin_bg(state)
    if not state.wait_ready():
        print("ERROR: no /joint_states carrying finger_joint")
        return 1

    exec_node = Node("rg2_stiffness_sweep_exec")
    backend = create_ros_moveit_backend(node=exec_node)
    action_name = _resolve_guarded_move_action_name(backend)
    gnode = backend.gripper.node

    rows: list[dict] = []
    try:
        # ---- park the arm once ------------------------------------------
        object_pose = tf.pose(tf_frame)
        offset = _load_grasp_center_offset(oid)
        if offset is not None:
            grasp_pose = compose_poses(object_pose, make_pose(
                position=offset.get("position", {"x": 0.0, "y": 0.0, "z": 0.0}),
                orientation=offset.get("orientation",
                                       {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})))
        else:
            grasp_pose = make_pose(position=dict(object_pose["position"]),
                                   orientation=dict(TOP_DOWN_QUAT),
                                   frame_id=WORLD_FRAME)
        pre = make_pose(
            position={**grasp_pose["position"],
                      "z": grasp_pose["position"]["z"] + APPROACH_HEIGHT_M},
            orientation=dict(grasp_pose["orientation"]), frame_id=WORLD_FRAME)

        print("parking the arm at the grasp pose (once)...")
        move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(backend, pre, gripper_width_m=width)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(backend, grasp_pose, gripper_width_m=width)
        time.sleep(SETTLE_SEC)

        hdr = (f"{'k1':>8}{'k2':>7}{'stop_reason':>17}{'settled_eff':>12}"
               f"{'width_mm':>10}{'d_width':>9}{'ROK_mm':>8}{'ROK_eff':>9}")
        print("\n" + hdr)
        print("-" * len(hdr))

        for k1, v1, k2, v2 in cells:
            if args.grid == "mimic":
                set_mimic(natfreq=v1, dampratio=v2)
            else:
                set_drive(stiffness=v1, damping=v2)
            time.sleep(0.4)

            backend.gripper.open()
            time.sleep(POST_OPEN_SEC)
            try:
                res = _call_guarded_move(
                    gnode, action_name=action_name,
                    target_width_m=target_width,
                    force_threshold=args.force_threshold, mode="torque")
                stop_reason = res.stop_reason
                final_width = float(res.final_width_m)
            except Exception as exc:  # noqa: BLE001
                print(f"{v1:8.0f}{v2:7.2f}{'EXCEPTION':>17}  {type(exc).__name__}: {exc}")
                rows.append({k1: v1, k2: v2, "stop_reason": "exception",
                             "error": str(exc)})
                continue

            time.sleep(HOLD_SEC)  # settle, then measure steady state
            pos, eff = state.snapshot()
            fj_pos = pos.get(FINGER_JOINT, float("nan"))
            fj_eff = eff.get(FINGER_JOINT, float("nan"))
            rok_resid = pos.get("right_outer_knuckle_joint", float("nan")) + fj_pos
            rok_eff = eff.get("right_outer_knuckle_joint", float("nan"))

            print(f"{v1:8.0f}{v2:7.2f}{stop_reason:>17}{fj_eff:12.2f}"
                  f"{final_width*1000:10.2f}{(final_width-width)*1000:+9.2f}"
                  f"{rok_resid*75:8.3f}{rok_eff:9.2f}")

            row = {k1: v1, k2: v2, "stop_reason": stop_reason,
                   "settled_finger_effort": fj_eff,
                   "final_width_m": final_width,
                   "width_error_m": final_width - width,
                   "finger_pos_rad": fj_pos,
                   "rok_residual_rad": rok_resid,
                   "rok_effort": rok_eff, "error": ""}
            for j in GRIPPER_JOINTS:
                row[f"{j}.pos"] = pos.get(j, "")
                row[f"{j}.eff"] = eff.get(j, "")
            rows.append(row)

            backend.gripper.open()
            time.sleep(POST_OPEN_SEC)

    finally:
        # Physics parameters are global to the stage: leaving a swept value
        # behind would silently change every later run, so restore even on
        # Ctrl-C or a mid-sweep exception.
        print("\nrestoring authored physics parameters...")
        try:
            restore(saved)
            print(f"  restored: {read_params()}")
        except Exception as exc:  # noqa: BLE001
            print(f"  !! RESTORE FAILED: {exc}\n  !! restore by hand or reload the stage")
        try:
            backend.gripper.open()
        except Exception:
            pass

    if rows:
        cols = sorted({k for r in rows for k in r})
        with open(csv_path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {len(rows)} cells to {csv_path}")

    print("\nHow to read this:")
    print("  settled_eff is the clamp AFTER the hold -- the force that actually")
    print("  holds the part. d_width near 0 means the fingers stopped at the")
    print("  part's surface; increasingly NEGATIVE d_width means they are")
    print("  penetrating a rigid part, i.e. over-clamping.")
    print("  ROK_mm is right_outer_knuckle's deviation from the linkage")
    print("  relation: it is the commanded travel being absorbed by constraint")
    print("  yield instead of reaching the part. Smaller is firmer.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
