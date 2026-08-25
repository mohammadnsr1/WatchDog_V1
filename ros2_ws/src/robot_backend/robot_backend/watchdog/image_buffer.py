"""Retrospective image ring buffer for WatchDog visual evidence.

WatchDog cannot predict when a monitor will fire, and the frames worth having
are the ones from BEFORE the fire. So rather than start recording when a step
begins, a small ring buffer holds recent frames continuously and the
capture_frames tool asks for a window retrospectively once something has
happened -- the same shape as get_evidence over the joint/pose buffers.

This module holds only the data structure: a deque of frames bounded by BOTH a
frame count and a total byte budget, evicting oldest-first when either bound is
reached. A byte bound is required because frame size varies with encoding and a
count-only bound cannot protect process memory. Every frame is stamped with
wall_ns() at receipt -- the same axis as every other buffer -- never with the
message header.stamp, which is simulation time and resets to zero each run.

ROS wiring (subscriptions, callback group) lives in the WatchDog MCP server;
keeping the buffer itself ROS-free makes it unit-testable with no cell.
"""

from __future__ import annotations

import collections
import os
import threading
from dataclasses import dataclass
from typing import Any

from .timebase import wall_ns

# Candidate RGB image topics, highest priority first. The RAW topic comes first
# on purpose: captures are written as PNG, and encoding a PNG needs raw pixels.
# A compressed (JPEG) frame would need a real codec to decode first, and this
# cell has neither OpenCV nor Pillow -- it would land on disk as bytes nobody
# can open. Raw costs far more ring-buffer memory per frame, which is the
# deliberate trade: fewer frames retained, but every one of them viewable.
# There is no depth *image* topic on this cell (only a point cloud, which is not
# an image and is out of scope for this buffer). Override with a comma-separated
# WATCHDOG_IMAGE_TOPICS to point at a different camera.
_DEFAULT_IMAGE_TOPIC_CANDIDATES = (
    "/camera/depth/rgb",
    "/camera/depth/rgb/compressed",
)


@dataclass
class ImageFrame:
    """One buffered camera frame, stamped on the wall_ns axis at receipt."""

    wall_ns: int
    topic: str
    encoding: str
    data: bytes
    width: int = 0
    height: int = 0

    @property
    def nbytes(self) -> int:
        return len(self.data)


class ImageRingBuffer:
    """Thread-safe ring buffer bounded by frame count and total bytes.

    Eviction is oldest-first and happens whenever appending would exceed either
    bound. A single frame larger than the byte budget is still stored (evicting
    everything else) rather than rejected -- a window that can return one frame
    is more useful than one that returns none, and dropping the newest frame
    silently would be the worse failure.
    """

    def __init__(self, max_frames: int, max_bytes: int) -> None:
        if max_frames <= 0:
            raise ValueError("max_frames must be positive.")
        if max_bytes <= 0:
            raise ValueError("max_bytes must be positive.")
        self.max_frames = int(max_frames)
        self.max_bytes = int(max_bytes)
        self._frames: collections.deque[ImageFrame] = collections.deque()
        self._total_bytes = 0
        self._lock = threading.Lock()

    def append(self, frame: ImageFrame) -> None:
        with self._lock:
            self._frames.append(frame)
            self._total_bytes += frame.nbytes
            self._evict_locked()

    def _evict_locked(self) -> None:
        # Count bound first, then byte bound. Keep at least one frame so a
        # single oversized frame is retained rather than looping to empty.
        while len(self._frames) > self.max_frames:
            self._total_bytes -= self._frames.popleft().nbytes
        while self._total_bytes > self.max_bytes and len(self._frames) > 1:
            self._total_bytes -= self._frames.popleft().nbytes

    def select_window(self, start_wall_ns: int, end_wall_ns: int) -> list[ImageFrame]:
        """Frames whose wall_ns falls in [start, end], oldest-first."""
        with self._lock:
            return [
                f for f in self._frames
                if start_wall_ns <= f.wall_ns <= end_wall_ns
            ]

    def buffered_range(self) -> list[int] | None:
        """[first, last] wall_ns currently buffered, or None if empty."""
        with self._lock:
            if not self._frames:
                return None
            return [self._frames[0].wall_ns, self._frames[-1].wall_ns]

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                "frame_count": len(self._frames),
                "total_bytes": self._total_bytes,
                "max_frames": self.max_frames,
                "max_bytes": self.max_bytes,
            }

    def __len__(self) -> int:
        with self._lock:
            return len(self._frames)


def _candidate_topics_from_env() -> tuple[str, ...]:
    raw = os.environ.get("WATCHDOG_IMAGE_TOPICS")
    if raw and raw.strip():
        return tuple(t.strip() for t in raw.split(",") if t.strip())
    return _DEFAULT_IMAGE_TOPIC_CANDIDATES


def _base_stream(topic: str) -> str:
    """Collapse a compressed variant to its raw stream name for grouping."""
    return topic[: -len("/compressed")] if topic.endswith("/compressed") else topic


class ImageBufferManager:
    """Subscribes to camera image topics and feeds an ImageRingBuffer.

    Throttles in the callback (no topic_tools node) by dropping frames that
    arrive faster than the target period, and stamps every accepted frame with
    wall_ns() at receipt -- never header.stamp, which is sim time. ROS imports
    are lazy so this module stays importable (and its throttle unit-testable)
    with no ROS present.

    The subscriptions must be created in a callback group SEPARATE from the
    recorder's /joint_states and TF callbacks: image callbacks are slow and
    must never delay monitor evaluation or halt latency. The caller passes that
    group in.
    """

    def __init__(
        self, buffer: ImageRingBuffer, hz: float = 5.0, logger: Any | None = None
    ) -> None:
        self.buffer = buffer
        self.hz = float(hz)
        self.min_period_ns = int(1_000_000_000 / self.hz) if self.hz > 0 else 0
        self.logger = logger
        self._last_accept_ns: dict[str, int] = {}
        self._subs: list[Any] = []
        self.subscribed_topics: list[str] = []

    def should_accept(self, topic: str, now_wall_ns: int) -> bool:
        """Throttle decision: accept at most one frame per min_period per topic."""
        last = self._last_accept_ns.get(topic)
        if last is None or (now_wall_ns - last) >= self.min_period_ns:
            self._last_accept_ns[topic] = now_wall_ns
            return True
        return False

    @staticmethod
    def choose_topics(
        candidates: tuple[str, ...] | list[str],
        types_by_topic: dict[str, list[str]],
    ) -> list[str]:
        """Pick the highest-priority existing topic per stream (prefer compressed).

        candidates are in priority order (compressed first). At most one topic
        per base stream is chosen, so a raw and its compressed variant never
        both get subscribed.
        """
        chosen: list[str] = []
        seen_bases: set[str] = set()
        for topic in candidates:
            if topic not in types_by_topic:
                continue
            base = _base_stream(topic)
            if base in seen_bases:
                continue
            seen_bases.add(base)
            chosen.append(topic)
        return chosen

    def attach(
        self,
        node: Any,
        callback_group: Any,
        candidates: tuple[str, ...] | list[str] | None = None,
    ) -> list[str]:
        """Discover and subscribe to available image topics. Returns chosen topics.

        If no image topic exists, logs once and returns [] -- an empty buffer is
        a supported configuration, not an error.
        """
        try:
            from sensor_msgs.msg import CompressedImage, Image
        except Exception as exc:  # pragma: no cover - needs ROS
            self._log(f"image buffer: sensor_msgs import failed ({exc}); no frames buffered")
            return []

        cand = tuple(candidates) if candidates is not None else _candidate_topics_from_env()
        try:
            types_by_topic = {t: types for t, types in node.get_topic_names_and_types()}
        except Exception as exc:  # pragma: no cover - needs ROS
            self._log(f"image buffer: topic discovery failed ({exc}); no frames buffered")
            return []

        chosen = self.choose_topics(cand, types_by_topic)
        if not chosen:
            self._log(
                f"image buffer: none of {list(cand)} are published; "
                "no visual evidence will be buffered (supported configuration)"
            )
            return []

        for topic in chosen:
            type_names = types_by_topic.get(topic, [])
            is_compressed = any("CompressedImage" in t for t in type_names)
            msg_cls = CompressedImage if is_compressed else Image
            kind = "compressed" if is_compressed else "raw"
            self._subs.append(
                node.create_subscription(
                    msg_cls, topic, self._make_callback(topic, kind), 5,
                    callback_group=callback_group,
                )
            )
            self.subscribed_topics.append(topic)
        self._log(f"image buffer: subscribed to {self.subscribed_topics} at {self.hz} Hz")
        return self.subscribed_topics

    def _make_callback(self, topic: str, kind: str):
        def _cb(msg: Any) -> None:
            try:
                now = wall_ns()
                if not self.should_accept(topic, now):
                    return
                data = bytes(getattr(msg, "data", b"") or b"")
                if kind == "compressed":
                    encoding = getattr(msg, "format", "compressed")
                    width = height = 0
                else:
                    encoding = getattr(msg, "encoding", "")
                    width = int(getattr(msg, "width", 0) or 0)
                    height = int(getattr(msg, "height", 0) or 0)
                self.buffer.append(
                    ImageFrame(
                        wall_ns=now, topic=topic, encoding=encoding,
                        data=data, width=width, height=height,
                    )
                )
            except Exception as exc:  # never let a bad frame kill the callback
                self._log(f"image buffer: frame callback failed on {topic}: {exc}")
        return _cb

    def _log(self, message: str) -> None:
        if self.logger is not None:
            try:
                self.logger.info(message)
            except Exception:
                pass


__all__ = ["ImageFrame", "ImageRingBuffer", "ImageBufferManager"]
