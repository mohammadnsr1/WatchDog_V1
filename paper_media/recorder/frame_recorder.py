#!/usr/bin/env python3
"""Passive frame recorder for paper media.

A standalone rclpy node that saves Isaac Sim camera frames, stamped with the
wall clock at receipt, into the active session directory. It is invisible to
both agents: it is not an MCP tool, it is in no --mcp-config file, and it only
subscribes. It never publishes and never calls a service.

Output, per camera:
    <session>/frames/<camera_name>/<index:06d>.jpg
    <session>/frames/<camera_name>/frames.jsonl   one line per saved frame:
        index, file, wall_time_iso (UTC, wall clock at receipt),
        header_stamp (simulation time from the message, reference only),
        topic, camera_name, width, height

The look() frames the agents capture live directly in <session>/frames/
(look_0001_rgb.png, exec_look_0001_rgb.png) and capture_frames() windows in
<session>/frames/<label>_<start>_<end>/. This node writes only into its own
<camera_name> subfolder, so it never collides with them.

Session directory resolution, in order:
    1. the out_dir parameter, if set (used as given; camera_name is appended)
    2. $WATCHDOG_SESSION_DIR/frames
    3. the directory named in ~/.watchdog/current_session, plus /frames
If none resolves, the node logs an error and exits without writing anything.

Timing: wall_time_iso is the only cross-process ordering key. header.stamp is
simulation time, reset to zero each run, and is stored for reference only.

Parameters:
    image_topic   default /recorder_rgb: the recording-only camera added to the
                  stage on 2026-10-29 (1280 x 720, fx 1826.39, frame sim_camera,
                  intrinsics on /recorder_camera_info). No agent reads it. Pass
                  /camera/depth/rgb to record the agents' own 640 x 360 view.
                  A topic ending in /compressed is subscribed as
                  CompressedImage and its JPEG bytes are written unchanged.
    out_dir       default ""  (resolve as above)
    camera_name   default "recorder"   (frames/recorder/; use "main" for /camera/depth/rgb)
    rate_hz       default 10.0   wall-clock throttle
    jpeg_quality  default 95
    max_width     default 1280   downscale wider raw frames, keep aspect

Run (ROS 2 Python, alongside a session):
    python3 paper_media/recorder/frame_recorder.py
    python3 paper_media/recorder/frame_recorder.py --ros-args \
        -p image_topic:=/camera/depth/rgb -p camera_name:=main
"""
from __future__ import annotations

import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_IMAGE_TOPIC = "/recorder_rgb"

# sensor_msgs/Image encodings handled for raw topics: encoding -> (Pillow mode, rawmode)
_RAW_MODES = {
    "rgb8": ("RGB", "RGB"),
    "bgr8": ("RGB", "BGR"),
    "rgba8": ("RGBA", "RGBA"),
    "bgra8": ("RGBA", "BGRA"),
    "mono8": ("L", "L"),
}


def wall_iso_from_ns(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).isoformat()


def resolve_session_dir() -> Optional[Path]:
    """$WATCHDOG_SESSION_DIR, else ~/.watchdog/current_session, else None."""
    env = os.environ.get("WATCHDOG_SESSION_DIR", "").strip()
    if env:
        return Path(env).expanduser()
    pointer = Path.home() / ".watchdog" / "current_session"
    try:
        raw = pointer.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    if not raw:
        return None
    candidate = Path(raw).expanduser()
    return candidate if candidate.is_dir() else None


class FrameRecorder(Node):
    def __init__(self) -> None:
        super().__init__("paper_media_frame_recorder")
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.declare_parameter("out_dir", "")
        self.declare_parameter("camera_name", "recorder")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("max_width", 1280)

        self.topic = str(self.get_parameter("image_topic").value)
        self.camera_name = str(self.get_parameter("camera_name").value).strip() or "main"
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.period_ns = int(1e9 / rate_hz) if rate_hz > 0 else 0
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.max_width = int(self.get_parameter("max_width").value)
        self.ok = False

        out_param = str(self.get_parameter("out_dir").value).strip()
        if out_param:
            base = Path(out_param).expanduser()
        else:
            session = resolve_session_dir()
            if session is None:
                self.get_logger().error(
                    "No output directory: out_dir is unset, WATCHDOG_SESSION_DIR is unset, "
                    "and ~/.watchdog/current_session does not name an existing directory. "
                    "Start the cell bringup first, or pass -p out_dir:=<dir>. Exiting."
                )
                return
            base = session / "frames"
        self.out_dir = base / self.camera_name
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.index_path = self.out_dir / "frames.jsonl"
        self.next_index = self._resume_index()
        self.last_saved_ns = 0
        self.n_dropped_encoding = 0

        self.compressed = self.topic.rstrip("/").endswith("/compressed")
        if self.compressed:
            from sensor_msgs.msg import CompressedImage as MsgType
            self._pil = None
        else:
            from sensor_msgs.msg import Image as MsgType
            try:
                from PIL import Image as PilImage  # noqa: N813
            except ImportError:
                self.get_logger().error(
                    "Pillow is not importable in this ROS 2 Python, so raw frames cannot be "
                    "encoded. Install it (sudo apt install python3-pil) or record the "
                    f"compressed topic ({self.topic}/compressed). Exiting."
                )
                return
            self._pil = PilImage

        # Best effort, depth 1: never back-pressure the simulator.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(MsgType, self.topic, self._on_msg, qos)
        self._index_fh = self.index_path.open("a", encoding="utf-8", buffering=1)
        self.ok = True
        self.get_logger().info(
            f"Recording {self.topic} at <= {rate_hz:g} Hz into {self.out_dir} "
            f"(starting at index {self.next_index})."
        )

    def _resume_index(self) -> int:
        """Continue numbering after an existing frames.jsonl so a restart never overwrites."""
        if not self.index_path.exists():
            return 0
        last = -1
        with self.index_path.open(encoding="utf-8") as fh:
            for line in fh:
                try:
                    last = max(last, int(json.loads(line)["index"]))
                except (ValueError, KeyError, json.JSONDecodeError):
                    continue
        return last + 1

    def _on_msg(self, msg) -> None:
        now_ns = time.time_ns()  # wall clock at receipt
        if self.period_ns and now_ns - self.last_saved_ns < self.period_ns:
            return
        name = f"{self.next_index:06d}.jpg"
        path = self.out_dir / name
        width = height = None
        try:
            if self.compressed:
                path.write_bytes(bytes(msg.data))
            else:
                width, height = self._write_raw(msg, path)
        except Exception as exc:  # noqa: BLE001  a bad frame must not stop recording
            self.n_dropped_encoding += 1
            if self.n_dropped_encoding <= 5:
                self.get_logger().warning(f"Frame not saved ({type(exc).__name__}: {exc}).")
            return
        stamp = msg.header.stamp
        record = {
            "index": self.next_index,
            "file": name,
            "wall_time_iso": wall_iso_from_ns(now_ns),
            "header_stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec),
                             "seconds": stamp.sec + stamp.nanosec * 1e-9},
            "topic": self.topic,
            "camera_name": self.camera_name,
            "width": width,
            "height": height,
        }
        self._index_fh.write(json.dumps(record) + "\n")
        self.last_saved_ns = now_ns
        self.next_index += 1

    def _write_raw(self, msg, path: Path) -> tuple[int, int]:
        enc = str(msg.encoding).lower()
        if enc not in _RAW_MODES:
            raise ValueError(f"unsupported encoding {msg.encoding!r}")
        mode, rawmode = _RAW_MODES[enc]
        img = self._pil.frombuffer(mode, (msg.width, msg.height), bytes(msg.data),
                                   "raw", rawmode, msg.step, 1)
        if img.mode != "RGB" and img.mode != "L":
            img = img.convert("RGB")
        if self.max_width > 0 and img.width > self.max_width:
            h = round(img.height * self.max_width / img.width)
            img = img.resize((self.max_width, h), self._pil.BILINEAR)
        img.save(path, "JPEG", quality=self.jpeg_quality)
        return img.width, img.height

    def close(self) -> None:
        fh = getattr(self, "_index_fh", None)
        if fh is not None:
            fh.close()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FrameRecorder()
    code = 0
    try:
        if node.ok:
            rclpy.spin(node)
        else:
            code = 1
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
