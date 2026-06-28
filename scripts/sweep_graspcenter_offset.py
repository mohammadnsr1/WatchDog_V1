"""Sweep widths, look up both contact frames RELATIVE TO TCP, average to the
grasp center, and record width -> tcp_to_graspcenter offset.

This produces the measured replacement for the linear tcp_offset_from_width
model, sharing one geometric source with the validated width table. It does
NOT assume anything about arm pose: by looking up the contact frames relative
to `tcp`, the offset is invariant to where the arm is parked.

Correctness checks built in:
  - grasp-center x/y in TCP frame should be ~0 (symmetry). Large values flag a
    sign/axis mismatch between the two contact Xforms.
  - the contact frames MUST move as width changes. If the recorded offset is
    constant across widths, the contact frames are static World transforms and
    are not tracking the fingers — fix the publisher first.

Run with Isaac up, gripper controller alive, contact frames in the TF tree:
    python3 sweep_graspcenter_offset.py --out graspcenter_offset.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
import time

import rclpy
from rclpy.node import Node
import tf2_ros

from robot_backend.ros_moveit_backend import create_ros_moveit_backend


TCP_FRAME = "tcp"
LEFT_CONTACT = "left_contact_frame"
RIGHT_CONTACT = "right_contact_frame"


def _trans_in_tcp(buf, child):
    tf = buf.lookup_transform(TCP_FRAME, child, rclpy.time.Time())
    t = tf.transform.translation
    return (t.x, t.y, t.z)


def run(widths, settle, out_path) -> bool:
    if not rclpy.ok():
        rclpy.init()
    node = Node("sweep_graspcenter_offset")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    backend = create_ros_moveit_backend(node=node)

    # Wait for the tree to connect tcp <-> contact frames.
    ok_tf = False
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if buf.can_transform(TCP_FRAME, LEFT_CONTACT, rclpy.time.Time()) and \
           buf.can_transform(TCP_FRAME, RIGHT_CONTACT, rclpy.time.Time()):
            ok_tf = True
            break
    if not ok_tf:
        print("ERROR: tcp<->contact frames not connected in TF. Check names and "
              "that the contact frames share the tree with tcp.")
        node.destroy_node()
        return False

    rows = []
    print(f"  {'cmd_width':>10} {'gc_z_tcp':>10} {'gc_x_tcp':>10} "
          f"{'gc_y_tcp':>10} {'sep':>8}")
    for w in widths:
        backend.gripper.set_width(w)
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        try:
            lt = _trans_in_tcp(buf, LEFT_CONTACT)
            rt = _trans_in_tcp(buf, RIGHT_CONTACT)
        except Exception as exc:  # noqa: BLE001
            print(f"  lookup failed at width {w:.3f}: {exc}")
            continue

        gc = tuple(0.5 * (a + b) for a, b in zip(lt, rt))
        sep = sum((a - b) ** 2 for a, b in zip(lt, rt)) ** 0.5
        rows.append((w, gc[2], gc[0], gc[1], sep))
        print(f"  {w:10.4f} {gc[2]:10.4f} {gc[0]:10.4f} {gc[1]:10.4f} {sep:8.4f}")

    # Constant-offset detector.
    if rows:
        zspan = max(r[1] for r in rows) - min(r[1] for r in rows)
        sepspan = max(r[4] for r in rows) - min(r[4] for r in rows)
        print("-" * 60)
        print(f"  z offset span across widths: {zspan*1000:.1f} mm")
        print(f"  finger separation span:      {sepspan*1000:.1f} mm")
        if sepspan < 0.002:
            print("  WARNING: separation barely changes -> contact frames may be\n"
                  "  STATIC (not tracking fingers). Verify the publisher refreshes\n"
                  "  their world pose every tick before trusting this table.")
        maxxy = max(max(abs(r[2]), abs(r[3])) for r in rows)
        if maxxy > 0.005:
            print(f"  WARNING: |grasp-center x/y in tcp| up to {maxxy*1000:.1f} mm,\n"
                  "  expected ~0 by symmetry. Check the two Xform sign/axis setup.")

    with open(out_path, "w", newline="") as fh:
        wtr = csv.writer(fh)
        wtr.writerow(["cmd_width_m", "tcp_to_gc_z_m",
                      "tcp_to_gc_x_m", "tcp_to_gc_y_m", "finger_sep_m"])
        wtr.writerows(rows)
    print(f"\nWrote {len(rows)} rows to {out_path}")

    node.destroy_node()
    return True


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="graspcenter_offset.csv")
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--min", type=float, default=0.0)
    ap.add_argument("--max", type=float, default=0.10)
    ap.add_argument("--step", type=float, default=0.01)
    args = ap.parse_args()

    n = int(round((args.max - args.min) / args.step)) + 1
    widths = [round(args.min + i * args.step, 4) for i in range(n)]
    try:
        ok = run(widths, args.settle, args.out)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)
