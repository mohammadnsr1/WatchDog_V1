#!/usr/bin/env python3
"""Reconcile a process's per-process event JSONL against what actually
arrived on /watchdog/events, and report any seq gaps.

This is the mechanical half of the EventBus hardening in
robot_backend.events.bus: seq (a per-process monotonic counter, starting at
1) and subscriber_count are stamped onto every EventEnvelope at publish time,
specifically so a dropped event is provable rather than inferred. A UUID
event_id identifies an event but cannot reveal a gap; seq can -- a consumer
that sees 7, 8, 10 knows 9 was lost.

This should be a standard post-run check for every evaluation run: run it
against each process's own JSONL log (the file passed as WATCHDOG_EVENT_LOG /
WATCHDOG_MONITOR_LOG to that process) and a capture of what the topic
subscriber(s) actually received. A run with no gaps is not just "probably
fine" -- it is provably fine, on the same evidence the study already keeps.

Usage
-----
Capture what actually arrives on the topic (run this before or during the
agents, alongside them, for the duration of the run):

    python3 reconcile_event_seq.py capture --topic-capture capture.jsonl

Reconcile one or more per-process logs against that capture afterward:

    python3 reconcile_event_seq.py check \\
        --topic-capture capture.jsonl \\
        --process-log ~/.watchdog/events_scene.jsonl \\
        --process-log ~/.watchdog/events_gripper.jsonl \\
        --process-log ~/.watchdog/events_motion.jsonl \\
        --process-log ~/.watchdog/monitor_events.jsonl

Exit code is 0 if every process log reconciles with no gaps, 1 otherwise --
suitable for wiring into a CI-style post-run check.

Both the capture file and the per-process logs are the same format: one
EventEnvelope JSON object per line (this is exactly what EventBus.publish
writes to both destinations -- see robot_backend/events/bus.py). Reconciling
them is directly comparable because both are the same schema; no adapter
step is required.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with open(path, encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"warning: {path}:{line_no} is not valid JSON: {exc}", file=sys.stderr)
    return events


def reconcile_one(process_log: Path, topic_by_event_id: dict[str, dict[str, Any]]) -> bool:
    """Return True if the process log reconciles cleanly (no gaps)."""
    process_events = load_jsonl(process_log)
    if not process_events:
        print(f"{process_log}: empty or unreadable, nothing to reconcile")
        return True

    emitted_seqs = sorted(e["seq"] for e in process_events if "seq" in e)
    if not emitted_seqs:
        print(f"{process_log}: no events carry a seq field (pre-hardening log?)")
        return False

    missing: list[dict[str, Any]] = []
    received_seqs: set[int] = set()
    for e in process_events:
        event_id = e.get("event_id")
        seq = e.get("seq")
        if event_id in topic_by_event_id:
            received_seqs.add(seq)
        else:
            missing.append(e)

    max_seq = max(emitted_seqs)
    expected = set(range(1, max_seq + 1))
    gaps = sorted(expected - received_seqs)

    print(f"\n{process_log}:")
    print(f"  emitted (per-process log): {len(process_events)} events, seq 1..{max_seq}")
    print(f"  confirmed on topic:        {len(received_seqs)} events")
    if not gaps:
        print("  gaps: none -- every emitted event was confirmed received")
        return True

    print(f"  gaps: {len(gaps)} missing seq value(s): {gaps}")
    for e in missing:
        print(
            f"    DROPPED seq={e.get('seq')} event_type={e.get('event_type')} "
            f"event_id={e.get('event_id')} subscriber_count_at_publish="
            f"{e.get('subscriber_count')}"
        )
    return False


def cmd_check(args: argparse.Namespace) -> int:
    topic_events = load_jsonl(Path(args.topic_capture))
    topic_by_event_id = {
        e["event_id"]: e for e in topic_events if "event_id" in e
    }
    print(f"topic capture: {len(topic_events)} events read from {args.topic_capture}")

    all_clean = True
    for process_log in args.process_log:
        clean = reconcile_one(Path(process_log), topic_by_event_id)
        all_clean = all_clean and clean

    print()
    if all_clean:
        print("RECONCILE OK: no seq gaps in any process log.")
        return 0
    print("RECONCILE FAILED: at least one process log has a seq gap. See DROPPED lines above.")
    return 1


def cmd_capture(args: argparse.Namespace) -> int:
    """Subscribe to /watchdog/events and append every message.data line to a file.

    Run this for the duration of a paired run (or longer) to get the topic
    side of the reconciliation. Ctrl-C to stop.
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    out_path = Path(args.topic_capture)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node("event_seq_capture")
    count = 0

    with open(out_path, "a", encoding="utf-8", buffering=1) as f:
        def _on_msg(msg: String) -> None:
            nonlocal count
            f.write(msg.data + "\n")
            count += 1

        node.create_subscription(String, "/watchdog/events", _on_msg, 1000)
        print(f"capturing /watchdog/events -> {out_path} (Ctrl-C to stop)")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            print(f"captured {count} events")
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_check = sub.add_parser("check", help="reconcile process logs against a topic capture")
    p_check.add_argument("--topic-capture", required=True, help="JSONL capture of /watchdog/events")
    p_check.add_argument(
        "--process-log", action="append", required=True,
        help="a per-process EventBus JSONL log; repeat for multiple processes",
    )
    p_check.set_defaults(func=cmd_check)

    p_capture = sub.add_parser("capture", help="record /watchdog/events to a JSONL file")
    p_capture.add_argument("--topic-capture", required=True, help="output path")
    p_capture.set_defaults(func=cmd_capture)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
