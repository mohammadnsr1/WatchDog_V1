"""Log master/finger joint effort from /joint_states over time.

Run this BEFORE (or just as) you trigger a grasp from the agent. It logs every
/joint_states sample for the chosen joint to stdout and to a CSV, with elapsed
time and the time-since-last-sample (so you can also see the ~286 ms stall
gaps in your /joint_states feed). Stop with Ctrl-C; it prints a summary
(contact effort guess = max effort, and the first timestamp effort crossed a
few candidate thresholds) and writes the CSV regardless.

Run:
    python3 log_grasp_effort.py --joint finger_joint --out grasp_effort_log.csv

Then in another terminal/agent turn, trigger the grasp. Ctrl-C this script
once the grasp finishes (or after a couple seconds of settle) to get the
summary and the CSV for plotting.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EffortLogger(Node):
    def __init__(self, joint_name: str, out_path: str, candidate_thresholds: list[float]):
        super().__init__("log_grasp_effort")
        self._joint_name = joint_name
        self._out_path = out_path
        self._candidates = sorted(candidate_thresholds)
        self._rows: list[tuple[float, float, float, float]] = []  # t, effort, dt_since_last_msg, position
        self._t0: float | None = None
        self._last_msg_time: float | None = None
        self._crossed: dict[float, float] = {}  # threshold -> elapsed time first crossed

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._cb, 50
        )
        self.get_logger().info(
            f"Logging effort for joint='{joint_name}' -> {out_path}. Ctrl-C to stop."
        )

    def _cb(self, msg: JointState) -> None:
        if self._joint_name not in msg.name:
            return
        idx = msg.name.index(self._joint_name)
        if not msg.effort or idx >= len(msg.effort):
            return
        effort = float(msg.effort[idx])
        position = float(msg.position[idx]) if msg.position and idx < len(msg.position) else float("nan")

        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        elapsed = now - self._t0
        dt_since_last = (now - self._last_msg_time) if self._last_msg_time is not None else 0.0
        self._last_msg_time = now

        self._rows.append((elapsed, effort, dt_since_last, position))

        for thr in self._candidates:
            if thr not in self._crossed and effort >= thr:
                self._crossed[thr] = elapsed

        gap_flag = "  <-- STALL" if dt_since_last > 0.05 else ""
        print(f"t={elapsed:7.3f}s  effort={effort:+.4f}  pos={position:+.4f}  "
              f"dt_since_last={dt_since_last*1000:6.1f}ms{gap_flag}")

    def summarize_and_write(self) -> None:
        if not self._rows:
            print("No samples received for joint "
                  f"'{self._joint_name}'. Check the joint name and that "
                  "/joint_states is publishing.")
            return

        efforts = [r[1] for r in self._rows]
        dts = [r[2] for r in self._rows[1:]]
        max_effort = max(efforts, key=abs)
        max_gap = max(dts) if dts else 0.0

        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        print(f"  samples: {len(self._rows)}")
        print(f"  peak |effort|: {max_effort:+.4f}")
        print(f"  largest gap between samples: {max_gap*1000:.1f} ms")
        if self._candidates:
            print("  first time each candidate threshold was crossed:")
            for thr in self._candidates:
                if thr in self._crossed:
                    print(f"    effort >= {thr:.3f}  ->  t = {self._crossed[thr]:.3f}s")
                else:
                    print(f"    effort >= {thr:.3f}  ->  never reached")
        print("=" * 60)

        with open(self._out_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["elapsed_sec", "effort", "dt_since_last_msg_sec", "position_rad"])
            w.writerows(self._rows)
        print(f"Wrote {len(self._rows)} rows to {self._out_path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="finger_joint",
                    help="Joint name to log effort for (master joint, e.g. finger_joint).")
    ap.add_argument("--out", default="grasp_effort_log.csv")
    ap.add_argument(
        "--thresholds",
        type=float,
        nargs="*",
        default=[0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.45],
        help="Candidate effort_contact_threshold values to report first-crossing time for.",
    )
    args = ap.parse_args()

    rclpy.init()
    node = EffortLogger(args.joint, args.out, args.thresholds)

    def _on_sigint(signum, frame):  # noqa: ARG001
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summarize_and_write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
