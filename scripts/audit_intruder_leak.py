#!/usr/bin/env python3
"""Audit a session for intruder-name leaks: audit_intruder_leak.py <session_dir>

Reads <session_dir>/ground_truth_injections.jsonl (written by
scripts/isaac/inject_intruder.py) and searches every channel an agent could
have learned an intruder's existence from WITHOUT the camera:

  (a) the run's rosbag /tf and /tf_static frame ids (via rosbag2_py when
      importable; otherwise says so in `checked` and relies on b-d),
  (b) mcp_calls_watchdog.jsonl and mcp_calls_executor.jsonl,
  (c) every events_*.jsonl,
  (d) authored_monitors/*.py,

for any mention of an intruder's name or its prim's leaf name OUTSIDE
look/observation text. Look and observation records are the legitimate
camera-mediated channel, so they are exempt: a look reason or an observation
summary may describe what the agent saw. Anything else naming an intruder --
a TF frame, a pose query, authored monitor code, a verdict explanation --
means the name reached the agent through a side channel, because the prim
name is never visible in pixels.

Output: {"leak": bool, "hits": [...], "checked": [...]} to stdout and to
<session_dir>/intruder_leak_audit.json. A run with leak=true is excluded from
observation scoring.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

GROUND_TRUTH_FILENAME = "ground_truth_injections.jsonl"
AUDIT_FILENAME = "intruder_leak_audit.json"

# Event types that are the legitimate camera-mediated channel (see module
# docstring). Everything else that names an intruder is a leak.
EXEMPT_EVENT_TYPES = {"look", "observation"}
# MCP tools whose call records are exempt for the same reason.
EXEMPT_TOOLS = {"look", "record_observation"}


def load_intruder_names(session_dir: Path) -> tuple[list[str], list[dict[str, Any]]]:
    """(sorted search terms, parsed ground-truth records)."""
    gt_path = session_dir / GROUND_TRUTH_FILENAME
    if not gt_path.is_file():
        raise FileNotFoundError(
            f"{gt_path} does not exist -- nothing was injected into this "
            "session, or the wrong directory was given."
        )
    names: set[str] = set()
    records: list[dict[str, Any]] = []
    for line in gt_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except ValueError:
            continue
        records.append(record)
        name = record.get("name")
        if isinstance(name, str) and name:
            names.add(name)
        prim_path = record.get("prim_path")
        if isinstance(prim_path, str) and prim_path:
            leaf = prim_path.rstrip("/").rsplit("/", 1)[-1]
            if leaf:
                names.add(leaf)
    return sorted(names), records


def _snippet(text: str, name: str, width: int = 90) -> str:
    index = text.find(name)
    start = max(0, index - width // 2)
    return text[start:start + width].strip()


def scan_jsonl_events(
    path: Path, names: list[str]
) -> list[dict[str, Any]]:
    """Hits in one events_*.jsonl, skipping look/observation event lines."""
    hits: list[dict[str, Any]] = []
    for line_no, line in enumerate(
        path.read_text(encoding="utf-8", errors="replace").splitlines(), 1
    ):
        stripped = line.strip()
        if not stripped or not any(name in stripped for name in names):
            continue
        event_type = None
        try:
            event = json.loads(stripped)
            if isinstance(event, dict):
                event_type = event.get("event_type")
        except ValueError:
            pass
        if event_type in EXEMPT_EVENT_TYPES:
            continue
        for name in names:
            if name in stripped:
                hits.append({
                    "source": "events", "file": path.name, "line": line_no,
                    "event_type": event_type, "name": name,
                    "snippet": _snippet(stripped, name),
                })
    return hits


def scan_jsonl_mcp_calls(path: Path, names: list[str]) -> list[dict[str, Any]]:
    """Hits in one mcp_calls_*.jsonl, skipping look/record_observation calls."""
    hits: list[dict[str, Any]] = []
    for line_no, line in enumerate(
        path.read_text(encoding="utf-8", errors="replace").splitlines(), 1
    ):
        stripped = line.strip()
        if not stripped or not any(name in stripped for name in names):
            continue
        tool = None
        try:
            record = json.loads(stripped)
            if isinstance(record, dict):
                tool = record.get("tool") or record.get("tool_name")
        except ValueError:
            pass
        if tool in EXEMPT_TOOLS:
            continue
        for name in names:
            if name in stripped:
                hits.append({
                    "source": "mcp_calls", "file": path.name, "line": line_no,
                    "tool": tool, "name": name,
                    "snippet": _snippet(stripped, name),
                })
    return hits


def scan_authored_monitors(session_dir: Path, names: list[str]) -> list[dict[str, Any]]:
    """Hits in authored_monitors/*.py -- any mention at all is a hit."""
    hits: list[dict[str, Any]] = []
    monitors_dir = session_dir / "authored_monitors"
    if not monitors_dir.is_dir():
        return hits
    for source in sorted(monitors_dir.glob("*.py")):
        text = source.read_text(encoding="utf-8", errors="replace")
        for line_no, line in enumerate(text.splitlines(), 1):
            for name in names:
                if name in line:
                    hits.append({
                        "source": "authored_monitors",
                        "file": f"authored_monitors/{source.name}",
                        "line": line_no, "name": name,
                        "snippet": line.strip()[:120],
                    })
    return hits


def find_bag_dirs(session_dir: Path) -> list[Path]:
    """Every rosbag2 directory under the session dir (has metadata.yaml)."""
    return sorted({p.parent for p in session_dir.rglob("metadata.yaml")})


def scan_bag(
    session_dir: Path, names: list[str]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """(hits, checked entries) for /tf and /tf_static frame ids in the bag."""
    hits: list[dict[str, Any]] = []
    checked: list[dict[str, Any]] = []
    bag_dirs = find_bag_dirs(session_dir)
    if not bag_dirs:
        checked.append({
            "source": "bag", "status": "unavailable: no rosbag2 directory "
            "(metadata.yaml) found under the session dir",
        })
        return hits, checked
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        checked.append({
            "source": "bag",
            "status": f"unavailable: rosbag2_py/tf2_msgs not importable "
            f"({exc}); falling back to the file-based channels (b-d)",
            "bag_dirs": [str(d) for d in bag_dirs],
        })
        return hits, checked

    for bag_dir in bag_dirs:
        seen_frames: set[str] = set()
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag_dir)),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            tf_topics = {
                t.name for t in reader.get_all_topics_and_types()
                if t.name in ("/tf", "/tf_static")
            }
            if tf_topics:
                reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(tf_topics)))
                while reader.has_next():
                    topic, data, _stamp = reader.read_next()
                    message = deserialize_message(data, TFMessage)
                    for transform in message.transforms:
                        seen_frames.add(str(transform.child_frame_id))
                        seen_frames.add(str(transform.header.frame_id))
            checked.append({
                "source": f"bag:{bag_dir.name}",
                "status": "scanned",
                "tf_topics": sorted(tf_topics),
                "distinct_frames": len(seen_frames),
            })
        except Exception as exc:
            checked.append({
                "source": f"bag:{bag_dir.name}",
                "status": f"error while reading: {type(exc).__name__}: {exc}",
            })
            continue
        for frame in sorted(seen_frames):
            for name in names:
                if name in frame:
                    hits.append({
                        "source": "bag_tf", "file": str(bag_dir),
                        "frame_id": frame, "name": name,
                    })
    return hits, checked


def audit_session(session_dir: Path) -> dict[str, Any]:
    """Run the whole audit. Raises only when the ground truth file is absent."""
    session_dir = Path(session_dir)
    names, records = load_intruder_names(session_dir)
    hits: list[dict[str, Any]] = []
    checked: list[dict[str, Any]] = []

    if not names:
        return {
            "leak": False, "hits": [],
            "checked": [{"source": "ground_truth",
                         "status": "file present but holds no intruder names"}],
            "intruder_names": [], "session_dir": str(session_dir),
        }

    bag_hits, bag_checked = scan_bag(session_dir, names)
    hits.extend(bag_hits)
    checked.extend(bag_checked)

    for pattern, scanner in (
        ("mcp_calls_*.jsonl", scan_jsonl_mcp_calls),
        ("events_*.jsonl", scan_jsonl_events),
    ):
        matched = sorted(session_dir.glob(pattern))
        if not matched:
            checked.append({"source": pattern, "status": "no files found"})
        for path in matched:
            try:
                hits.extend(scanner(path, names))
                checked.append({"source": path.name, "status": "scanned"})
            except Exception as exc:
                checked.append({
                    "source": path.name,
                    "status": f"error while reading: {type(exc).__name__}: {exc}",
                })

    monitors_dir = session_dir / "authored_monitors"
    if monitors_dir.is_dir():
        hits.extend(scan_authored_monitors(session_dir, names))
        checked.append({
            "source": "authored_monitors/*.py", "status": "scanned",
            "files": len(list(monitors_dir.glob("*.py"))),
        })
    else:
        checked.append({
            "source": "authored_monitors/*.py", "status": "no directory",
        })

    return {
        "leak": bool(hits),
        "hits": hits,
        "checked": checked,
        "intruder_names": names,
        "ground_truth_records": len(records),
        "session_dir": str(session_dir),
    }


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    session_dir = Path(argv[1]).expanduser()
    if not session_dir.is_dir():
        print(f"error: {session_dir} is not a directory", file=sys.stderr)
        return 2
    try:
        result = audit_session(session_dir)
    except FileNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    text = json.dumps(result, indent=2)
    print(text)
    (session_dir / AUDIT_FILENAME).write_text(text + "\n", encoding="utf-8")
    return 1 if result["leak"] else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
