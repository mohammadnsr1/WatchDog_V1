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

This module also holds CameraStreams, which is a different thing from the ring
buffer above and exists for a different consumer. The ring buffer keeps a
HISTORY of RGB frames so capture_frames can reach backwards to a moment that has
already passed. CameraStreams keeps only the LATEST message of each camera
stream, because look() answers "what does the workspace look like now" and a
history would be dead weight. The two coexist: the ring buffer's manager feeds
its accepted frames forward into CameraStreams (offer_rgb) rather than have a
second RGB subscription deserialize every frame twice.
"""

from __future__ import annotations

import collections
import functools
import math
import os
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from .timebase import wall_ns

# Candidate RGB image topics, highest priority first. The RAW topic comes first
# on purpose: captures are written as PNG, and encoding a PNG needs raw pixels.
# A compressed (JPEG) frame would need a real codec to decode first, and this
# cell has neither OpenCV nor Pillow -- it would land on disk as bytes nobody
# can open. Raw costs far more ring-buffer memory per frame, which is the
# deliberate trade: fewer frames retained, but every one of them viewable.
# This ring buffer is RGB-only and stays that way: it exists for capture_frames,
# whose whole job is preserving viewable colour stills of a moment that has
# passed. Depth arrives on its own topic (see _DEFAULT_DEPTH_IMAGE_TOPIC below)
# and is served latest-only through CameraStreams, because a float32 depth frame
# is 4 bytes per pixel and would evict colour frames from a shared byte budget.
# Override with a comma-separated WATCHDOG_IMAGE_TOPICS to point at a different
# camera.
_DEFAULT_IMAGE_TOPIC_CANDIDATES = (
    "/camera/depth/rgb",
    "/camera/depth/rgb/compressed",
)

# The remaining camera streams look() reads. Every one of these is optional and a
# missing topic degrades a view, never the process -- see CameraStreams.attach.
#
# Live on this cell as of 2026-09-30: /camera/depth/rgb, /camera/depth/image_raw,
# /camera/depth/camera_info. The POINT CLOUD is NOT published any more, so the
# projection fallback below never runs here -- see _DEFAULT_POINTCLOUD_TOPIC.
_DEFAULT_DEPTH_IMAGE_TOPIC = "/camera/depth/image_raw"
# Two candidates in priority order, because this cell has used two names.
# "/camera/depth/camera_info" is what the render product publishes now (verified
# live 2026-09-30) and is the one that will be picked. "/camera_info" (global,
# unnamespaced) is the older name, still referenced by sim_moveit_bringup's
# _IMAGE_BAG_TOPICS, and is kept here so an un-updated stage still resolves.
# First one present wins; subscribing to both would double-fill one slot from
# two sources with no way to say which intrinsics were used.
_DEFAULT_CAMERA_INFO_TOPICS = (
    "/camera/depth/camera_info",
    "/camera_info",
)
# Kept in step with sim_moveit_bringup/config/cameras.yaml's pointcloud_topic.
# Not read from that file: this process has no launch context and the yaml lives
# in a different package's share directory. Override with the env var below if
# the two ever diverge.
#
# NOTHING PUBLISHES THIS ON THE CURRENT CELL (verified 2026-09-30) and nothing
# subscribes to it either: use_octomap and filter_pointcloud both default false
# (sim_moveit_bringup.launch.py:507, :539), and that launch file's own note says
# with the filter off "the depth publisher can be switched off in Isaac too" --
# which is what happened. The cloud path here is therefore DORMANT, retained
# deliberately as the fallback if /camera/depth/image_raw ever regresses. It is
# consequently UNEXERCISED code: treat project_cloud_to_depth and cloud_xyz as
# unverified against a real message until a cloud is republished.
_DEFAULT_POINTCLOUD_TOPIC = "/camera/depth/points"

# How old the latest message of a stream may be, on the wall_ns receipt axis,
# before look() calls that view stale. One second is long for a 5-30 Hz camera
# and short relative to a step, so a stale view means something is actually
# wrong (Isaac paused, topic gone) rather than ordinary jitter.
DEFAULT_STALE_SEC = 1.0

# sensor_msgs/PointField datatype enum -> numpy dtype. Needed because this cell
# deliberately has no ros2_numpy / sensor_msgs_py dependency for the projection
# path; the cloud is parsed straight out of its bytes with numpy.
_POINTFIELD_DTYPES = {
    1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2",
    5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8",
}


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
        self,
        buffer: ImageRingBuffer,
        hz: float = 5.0,
        logger: Any | None = None,
        streams: Any | None = None,
    ) -> None:
        self.buffer = buffer
        self.hz = float(hz)
        self.min_period_ns = int(1_000_000_000 / self.hz) if self.hz > 0 else 0
        self.logger = logger
        # Optional CameraStreams to forward accepted frames to, so look()'s
        # latest-RGB slot is filled from this one subscription instead of a
        # second one. Default None keeps every existing caller unchanged.
        self.streams = streams
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
        """Subscribe to the configured image topics unconditionally. Returns them.

        Subscriptions are no longer gated on get_topic_names_and_types(): that
        races DDS discovery at server start, and a topic missed in the attach
        snapshot stayed unsubscribed for the whole run (observed live
        2026-09-30, alongside the same race in CameraStreams.attach). One topic
        per base stream is still chosen -- the highest-priority candidate --
        and a compressed variant is recognised by name or, when discovery has
        it, by type. An unpublished topic just never delivers, which is the
        supported empty-buffer configuration.
        """
        try:
            from sensor_msgs.msg import CompressedImage, Image
        except Exception as exc:  # pragma: no cover - needs ROS
            self._log(f"image buffer: sensor_msgs import failed ({exc}); no frames buffered")
            return []

        cand = tuple(candidates) if candidates is not None else _candidate_topics_from_env()
        # Diagnostic only (compressed-type detection for discovered topics);
        # never a condition on subscribing.
        try:
            types_by_topic = {t: types for t, types in node.get_topic_names_and_types()}
        except Exception:  # pragma: no cover - needs ROS
            types_by_topic = {}

        chosen: list[str] = []
        seen_bases: set[str] = set()
        for topic in cand:
            base = _base_stream(topic)
            if base in seen_bases:
                continue
            seen_bases.add(base)
            chosen.append(topic)

        for topic in chosen:
            type_names = types_by_topic.get(topic, [])
            is_compressed = topic.endswith("/compressed") or any(
                "CompressedImage" in t for t in type_names
            )
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
                frame = ImageFrame(
                    wall_ns=now, topic=topic, encoding=encoding,
                    data=data, width=width, height=height,
                )
                self.buffer.append(frame)
                if self.streams is not None:
                    # Forwarded AFTER the ring append so a failure here can
                    # never cost capture_frames its frame.
                    try:
                        self.streams.offer_rgb(frame)
                    except Exception as exc:
                        self._log(f"image buffer: offer_rgb failed on {topic}: {exc}")
            except Exception as exc:  # never let a bad frame kill the callback
                self._log(f"image buffer: frame callback failed on {topic}: {exc}")
        return _cb

    def _log(self, message: str) -> None:
        if self.logger is not None:
            try:
                self.logger.info(message)
            except Exception:
                pass


@dataclass
class _Latest:
    """One stream's most recent message, with both clocks kept separate.

    stamp_ns is the message header stamp -- SIM time, which resets to zero every
    run and is comparable only to other sim-stamped data within the same run.
    wall_ns is receipt time on the process's wall clock, which is the axis every
    event and evidence buffer in this system uses and the only one comparable
    across processes. Staleness is judged on wall_ns; alignment against TF is
    judged on stamp_ns. Conflating them is the bug this dataclass exists to make
    hard.
    """

    msg: Any
    stamp_ns: int
    wall_ns: int


def _header_stamp_ns(msg: Any) -> int:
    """Nanoseconds from a message's header.stamp, or 0 if it has none."""
    try:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except Exception:
        return 0


class CameraStreams:
    """Latest-only store for the camera streams look() reads.

    Four streams, all optional:
      rgb          -- fed by ImageBufferManager.offer_rgb, or by its own
                      subscription when no manager is present (the executor
                      case, which has no ring buffer).
      depth image  -- 32FC1 metres, the preferred depth source.
      camera_info  -- intrinsics; also the only way to size a projected cloud.
      point cloud  -- the depth FALLBACK, projected through camera_info when no
                      depth image is arriving.

    Every accessor returns None rather than raising when its inputs are absent
    or stale, and describe_depth_availability() explains a None in words a tool
    result can carry to the agent. A camera that is not publishing is a degraded
    look, never a failed run -- the same stance ImageBufferManager.attach takes
    for a missing RGB topic.

    Thread-safe and latest-only: one lock, four slots, no history. Subscriptions
    must go in a callback group separate from any monitor-evaluation callbacks,
    for the reason ImageBufferManager documents -- image work is slow and must
    not delay halt latency. The caller passes that group in.
    """

    STREAM_NAMES = ("rgb", "depth", "camera_info", "pointcloud")

    def __init__(
        self,
        logger: Any | None = None,
        stale_sec: float = DEFAULT_STALE_SEC,
    ) -> None:
        self.logger = logger
        self.stale_sec = float(stale_sec)
        self._lock = threading.Lock()
        self._rgb: _Latest | None = None
        self._depth: _Latest | None = None
        self._info: _Latest | None = None
        self._info_topic: str | None = None
        self._info_preferred: str = _DEFAULT_CAMERA_INFO_TOPICS[0]
        self._cloud: _Latest | None = None
        self._subs: list[Any] = []
        self.subscribed_topics: list[str] = []
        self.missing_topics: list[str] = []
        self.attached_wall_ns: int | None = None
        # Per-stream receive/error accounting. This is the instrumentation a
        # live "nothing arrives" failure explains itself with: every callback
        # counts here, every swallowed exception counts here, and stats()
        # serves it to look()'s text part. topics is what attach() actually
        # subscribed for the stream; topic is the one that last FILLED the
        # slot (they differ for camera_info, which subscribes two candidates).
        self._stats: dict[str, dict[str, Any]] = {
            name: {
                "topics": [], "topic": None, "qos_reliability": "RELIABLE",
                "received": 0, "last_wall_ns": None,
                "errors": 0, "first_error": None,
            }
            for name in self.STREAM_NAMES
        }

    # -- receive/error accounting -------------------------------------------

    def _count_received(self, stream: str, topic: str | None = None) -> None:
        with self._lock:
            entry = self._stats[stream]
            entry["received"] += 1
            entry["last_wall_ns"] = wall_ns()
            if topic:
                entry["topic"] = topic

    def _count_error(self, stream: str, exc: BaseException) -> None:
        """Count a callback/forward failure; log the first, then every 100th."""
        text = f"{type(exc).__name__}: {exc}"
        with self._lock:
            entry = self._stats[stream]
            entry["errors"] += 1
            count = entry["errors"]
            if entry["first_error"] is None:
                entry["first_error"] = text
        if count == 1 or count % 100 == 0:
            self._log(f"camera streams: {stream} callback error #{count}: {text}")

    # -- ingest ------------------------------------------------------------

    def offer_rgb(self, frame: ImageFrame) -> None:
        """Accept an RGB frame already received by ImageBufferManager.

        Takes an ImageFrame rather than a raw message because that is what the
        manager has, and re-deriving width/height/encoding from the message a
        second time would be the only alternative. The frame carries its own
        receipt wall_ns, so no clock is re-read here.
        """
        try:
            with self._lock:
                self._rgb = _Latest(msg=frame, stamp_ns=0, wall_ns=int(frame.wall_ns))
            self._count_received("rgb", getattr(frame, "topic", None))
        except Exception as exc:
            self._count_error("rgb", exc)

    def attach(
        self,
        node: Any,
        callback_group: Any,
        *,
        subscribe_rgb: bool = False,
        rgb_candidates: tuple[str, ...] | list[str] | None = None,
    ) -> dict[str, Any]:
        """Subscribe to whichever of the camera streams exist. Never raises.

        subscribe_rgb=False is the WatchDog case: ImageBufferManager already
        owns an RGB subscription and forwards through offer_rgb, so a second one
        would deserialize every frame twice for nothing. subscribe_rgb=True is
        the executor case, which has no ring buffer and needs its own.

        Subscriptions are created UNCONDITIONALLY for every configured topic.
        They used to be gated on get_topic_names_and_types(), which races DDS
        discovery: a server that attaches milliseconds after its node is
        created sees an empty graph, skips every topic, and look() is then
        blind for the whole run even though the camera was publishing all
        along (observed live 2026-09-30). Discovery is now DIAGNOSTIC only --
        it feeds missing_topics and the attach log line, never the decision
        to subscribe. camera_info subscribes BOTH candidate names; whichever
        fills the slot is recorded, and the namespaced one is preferred.
        Returns a dict describing what was wired.
        """
        result: dict[str, Any] = {
            "subscribed": [], "missing": [], "errors": [],
        }
        self.attached_wall_ns = wall_ns()
        try:
            from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        except Exception as exc:  # pragma: no cover - needs ROS
            self._log(f"camera streams: sensor_msgs import failed ({exc}); no views")
            result["errors"].append(f"sensor_msgs import failed: {exc}")
            return result

        # Diagnostic snapshot only. Empty or stale is fine: it changes what is
        # REPORTED, never what is subscribed.
        try:
            types_by_topic = {t: types for t, types in node.get_topic_names_and_types()}
        except Exception as exc:  # pragma: no cover - needs ROS
            types_by_topic = {}
            result["errors"].append(f"topic discovery failed (diagnostic only): {exc}")

        wanted: list[tuple[str, str, Any, Any]] = []

        if subscribe_rgb:
            cand = (
                tuple(rgb_candidates) if rgb_candidates is not None
                else _candidate_topics_from_env()
            )
            for topic in cand:
                looks_compressed = topic.rstrip("/").endswith("compressed") or any(
                    "CompressedImage" in t for t in types_by_topic.get(topic, [])
                )
                if looks_compressed:
                    # look() must hand the model real pixels, and decoding JPEG
                    # needs a codec this cell does not ship (see png_encode).
                    self._log(
                        f"camera streams: skipping compressed RGB topic {topic!r} "
                        "-- no codec in this interpreter to decode it"
                    )
                    result["missing"].append(
                        {"topic": topic, "reason": "compressed; no codec available"}
                    )
                    continue
                wanted.append(
                    ("rgb", topic, Image, functools.partial(self._on_rgb_msg, topic=topic))
                )

        depth_topic = os.environ.get("WATCHDOG_DEPTH_IMAGE_TOPIC", _DEFAULT_DEPTH_IMAGE_TOPIC)
        cloud_topic = os.environ.get("WATCHDOG_POINTCLOUD_TOPIC", _DEFAULT_POINTCLOUD_TOPIC)
        wanted.append((
            "depth", depth_topic, Image,
            functools.partial(self._on_depth, topic=depth_topic),
        ))
        info_candidates = self._info_candidates()
        self._info_preferred = info_candidates[0]
        for info_topic in info_candidates:
            wanted.append((
                "camera_info", info_topic, CameraInfo,
                functools.partial(self._on_info, topic=info_topic),
            ))
        wanted.append((
            "pointcloud", cloud_topic, PointCloud2,
            functools.partial(self._on_cloud, topic=cloud_topic),
        ))

        for stream, topic, msg_cls, handler in wanted:
            if topic not in types_by_topic:
                # Not yet discovered at attach time -- recorded for the log,
                # but subscribed anyway: DDS delivers once the publisher is
                # discovered, whenever that happens.
                self.missing_topics.append(topic)
                result["missing"].append(
                    {"topic": topic, "reason": "not discovered at attach (subscribed anyway)"}
                )
            try:
                self._subs.append(
                    node.create_subscription(
                        msg_cls, topic, handler, 1, callback_group=callback_group,
                    )
                )
            except Exception as exc:  # pragma: no cover - needs ROS
                result["errors"].append(f"{topic}: {exc}")
                continue
            self.subscribed_topics.append(topic)
            with self._lock:
                self._stats[stream]["topics"].append(topic)
            result["subscribed"].append(topic)

        self._log(
            f"camera streams: subscribed to {result['subscribed']}; "
            f"not discovered yet {[m['topic'] for m in result['missing']]}"
        )
        return result

    @staticmethod
    def _info_candidates() -> tuple[str, ...]:
        """camera_info topics to subscribe: env override alone, else both defaults.

        The first entry is the PREFERRED one: when both candidates publish, the
        slot follows the preferred (namespaced) topic.
        """
        override = os.environ.get("WATCHDOG_CAMERA_INFO_TOPIC")
        if override and override.strip():
            return (override.strip(),)
        return tuple(_DEFAULT_CAMERA_INFO_TOPICS)

    # Queue depth 1 on every subscription above: this is a latest-only store, so
    # a backlog would only ever be frames it is about to discard.

    def _on_rgb_msg(self, msg: Any, topic: str = "rgb") -> None:
        try:
            frame = ImageFrame(
                wall_ns=wall_ns(),
                topic=topic,
                encoding=str(getattr(msg, "encoding", "") or ""),
                data=bytes(getattr(msg, "data", b"") or b""),
                width=int(getattr(msg, "width", 0) or 0),
                height=int(getattr(msg, "height", 0) or 0),
            )
            with self._lock:
                self._rgb = _Latest(
                    msg=frame, stamp_ns=_header_stamp_ns(msg), wall_ns=frame.wall_ns
                )
            self._count_received("rgb", topic)
        except Exception as exc:
            self._count_error("rgb", exc)

    def _on_depth(self, msg: Any, topic: str | None = None) -> None:
        self._store("_depth", "depth", msg, topic)

    def _on_info(self, msg: Any, topic: str) -> None:
        """Store camera_info, preferring the namespaced topic when both publish."""
        try:
            latest = _Latest(msg=msg, stamp_ns=_header_stamp_ns(msg), wall_ns=wall_ns())
            with self._lock:
                if (
                    topic == self._info_preferred
                    or self._info is None
                    or self._info_topic != self._info_preferred
                ):
                    self._info = latest
                    self._info_topic = topic
            self._count_received("camera_info", topic)
        except Exception as exc:
            self._count_error("camera_info", exc)

    def _on_cloud(self, msg: Any, topic: str | None = None) -> None:
        self._store("_cloud", "pointcloud", msg, topic)

    def _store(
        self, attr: str, stream: str, msg: Any, topic: str | None = None
    ) -> None:
        try:
            latest = _Latest(msg=msg, stamp_ns=_header_stamp_ns(msg), wall_ns=wall_ns())
            with self._lock:
                setattr(self, attr, latest)
            self._count_received(stream, topic)
        except Exception as exc:
            self._count_error(stream, exc)

    # -- age helpers -------------------------------------------------------

    def _age_sec(self, latest: _Latest | None, now: int) -> float | None:
        if latest is None:
            return None
        return max(0.0, (now - latest.wall_ns) / 1e9)

    def ages(self) -> dict[str, float | None]:
        """Age in seconds of each stream's latest message, None if never seen."""
        now = wall_ns()
        with self._lock:
            slots = {
                "rgb": self._rgb, "depth": self._depth,
                "camera_info": self._info, "pointcloud": self._cloud,
            }
        return {name: self._age_sec(latest, now) for name, latest in slots.items()}

    # -- accessors ---------------------------------------------------------

    def get_rgb(self) -> tuple[Any, int] | None:
        """(HxWx3 uint8 ndarray, stamp_ns) for the latest RGB frame, or None.

        stamp_ns is the message header stamp (sim time) when the frame came
        through its own subscription, and 0 when it was forwarded from
        ImageBufferManager, which does not retain headers. Callers that need a
        cross-process time must use the age reported by ages(), not this.

        Returns None when no frame has arrived, when it is stale, or when its
        encoding is not one this cell can turn into pixels.
        """
        now = wall_ns()
        with self._lock:
            latest = self._rgb
        if latest is None:
            return None
        if (self._age_sec(latest, now) or 0.0) > self.stale_sec:
            return None
        frame = latest.msg
        array = rgb_to_array(frame.data, frame.width, frame.height, frame.encoding)
        if array is None:
            return None
        return array, latest.stamp_ns

    def get_depth(self) -> tuple[Any, int, str] | None:
        """(HxW float32 metres, stamp_ns, source) or None.

        source is "depth_image" when a fresh 32FC1 image is arriving, and
        "pointcloud_projection" when it is not and the latest cloud had to be
        projected through camera_info instead. The two are NOT equivalent and
        the difference is reported to the model: a depth image is dense and
        per-pixel, while a projection is sparse, carries only whatever points
        the cloud retained, and leaves NaN wherever nothing landed.

        Returns None when neither path is available; call
        describe_depth_availability() for the reason in words.
        """
        import numpy as np

        now = wall_ns()
        with self._lock:
            depth, info, cloud = self._depth, self._info, self._cloud

        depth_age = self._age_sec(depth, now)
        if depth is not None and depth_age is not None and depth_age <= self.stale_sec:
            array = depth_image_to_metres(depth.msg)
            if array is not None:
                return array, depth.stamp_ns, "depth_image"

        cloud_age = self._age_sec(cloud, now)
        if (
            cloud is not None
            and info is not None
            and cloud_age is not None
            and cloud_age <= self.stale_sec
        ):
            intrinsics = camera_info_to_intrinsics(info.msg)
            if intrinsics is not None:
                array = project_cloud_to_depth(cloud.msg, intrinsics)
                if array is not None and np.isfinite(array).any():
                    return array, cloud.stamp_ns, "pointcloud_projection"
        return None

    def array_snapshot(self) -> dict[str, Any]:
        """One coherent read of rgb + depth + camera for an array-lane tick.

        Returns {"taken_wall_ns", "rgb": (HxWx3 uint8 ndarray, recv_wall_ns) or
        None, "depth": (HxW float32 metres, recv_wall_ns) or None, "camera":
        get_camera() or None}. Both slots are read under ONE lock acquisition,
        so the pair is coherent; a frame older than stale_sec is returned as
        None, same staleness the view accessors apply. The arrays are fresh
        decodes with writeable=False -- authored code receives them directly
        and must not be able to corrupt a frame another monitor's snapshot
        would share.
        """
        import numpy as np  # local, matching get_depth

        now = wall_ns()
        with self._lock:
            rgb, depth = self._rgb, self._depth
        out: dict[str, Any] = {
            "taken_wall_ns": now, "rgb": None, "depth": None,
            "camera": self.get_camera(),
        }
        rgb_age = self._age_sec(rgb, now)
        if rgb is not None and rgb_age is not None and rgb_age <= self.stale_sec:
            frame = rgb.msg
            array = rgb_to_array(frame.data, frame.width, frame.height, frame.encoding)
            if array is not None:
                array = np.ascontiguousarray(array)
                array.flags.writeable = False
                out["rgb"] = (array, int(rgb.wall_ns))
        depth_age = self._age_sec(depth, now)
        if depth is not None and depth_age is not None and depth_age <= self.stale_sec:
            array = depth_image_to_metres(depth.msg)
            if array is not None:
                array = np.ascontiguousarray(array)
                array.flags.writeable = False
                out["depth"] = (array, int(depth.wall_ns))
        return out

    def describe_depth_availability(self) -> str:
        """Why get_depth() returned None, in one sentence for a tool result.

        Recomputed from the slots rather than remembered from the failed call,
        so it cannot describe a state that has since changed and cannot go stale
        behind the accessor.
        """
        ages = self.ages()
        depth_age, cloud_age, info_age = (
            ages["depth"], ages["pointcloud"], ages["camera_info"]
        )
        parts: list[str] = []
        if depth_age is None:
            parts.append(self.never_received_text("depth"))
        elif depth_age > self.stale_sec:
            parts.append(f"depth image is {depth_age:.2f}s old (stale > {self.stale_sec}s)")
        else:
            parts.append("depth image arrived but could not be read as 32FC1 metres")

        if info_age is None:
            parts.append(
                "and camera_info is absent, so the point-cloud fallback cannot be "
                "projected either (it needs fx, fy, cx, cy and the image size)"
            )
        elif cloud_age is None:
            parts.append("and no point cloud has arrived to project as a fallback")
        elif cloud_age > self.stale_sec:
            parts.append(
                f"and the point-cloud fallback is {cloud_age:.2f}s old "
                f"(stale > {self.stale_sec}s)"
            )
        else:
            parts.append(
                "and projecting the latest point cloud produced no valid pixels"
            )
        return "; ".join(parts) + "."

    def get_camera(self) -> dict[str, Any] | None:
        """Intrinsics + the yaml-sourced extrinsics, or None without camera_info.

        Extrinsics are the FIXED world -> camera_depth_optical_frame transform
        that sim_moveit_bringup publishes as a static TF from
        sim_moveit_bringup/config/cameras.yaml. They are reported here as the
        values that file declares, not as a TF lookup: this server has no TF
        buffer, and a static transform sourced from a file is already the
        authority for where the camera is said to be. Note that makes this the
        DECLARED pose -- if the camera prim in the USD stage disagrees with the
        yaml, nothing here detects it.

        Staleness is not applied: intrinsics do not change during a run, and a
        CameraInfo publisher that emits once at startup is normal.
        """
        with self._lock:
            info = self._info
        if info is None:
            return None
        intrinsics = camera_info_to_intrinsics(info.msg)
        if intrinsics is None:
            return None
        return {
            "frame_id": str(getattr(info.msg.header, "frame_id", "") or ""),
            "width": intrinsics["width"],
            "height": intrinsics["height"],
            "K": intrinsics["K"],
            "D": intrinsics["D"],
            "distortion_model": intrinsics["distortion_model"],
            "extrinsics_from_yaml": dict(CAMERA_EXTRINSICS_FROM_YAML),
            "stamp_ns": info.stamp_ns,
        }

    def stats(self) -> dict[str, Any]:
        """Per-stream receive/error accounting, for look()'s "streams" block.

        One entry per stream: the topic(s) subscribed, QoS reliability, how
        many messages arrived, when the last one arrived (wall clock), how
        many callback/forward errors occurred and the first error's text. A
        live "nothing arrives" failure reads its own explanation out of this.
        """
        now = wall_ns()
        ages = self.ages()
        with self._lock:
            streams = {
                name: dict(entry, topics=list(entry["topics"]))
                for name, entry in self._stats.items()
            }
            info_topic = self._info_topic
        for name, entry in streams.items():
            last = entry.get("last_wall_ns")
            entry["age_sec"] = (
                round((now - last) / 1e9, 3) if last is not None else ages.get(name)
            )
        if info_topic:
            streams["camera_info"]["topic"] = info_topic
        return {
            "attached_wall_ns": self.attached_wall_ns,
            "attached_iso": self._attached_iso(),
            "subscribed_topics": list(self.subscribed_topics),
            "not_discovered_at_attach": list(self.missing_topics),
            "ages_sec": ages,
            "stale_sec": self.stale_sec,
            "streams": streams,
        }

    def _attached_iso(self) -> str | None:
        if self.attached_wall_ns is None:
            return None
        return datetime.fromtimestamp(
            self.attached_wall_ns / 1e9, tz=timezone.utc
        ).isoformat()

    def never_received_text(self, stream: str) -> str:
        """The views_failed sentence for a stream that has never delivered."""
        with self._lock:
            entry = dict(self._stats.get(stream) or {})
            topics = list(entry.get("topics") or [])
        topic = ", ".join(topics) if topics else f"<no {stream} subscription>"
        if self.attached_wall_ns is not None:
            elapsed = f"{(wall_ns() - self.attached_wall_ns) / 1e9:.0f}"
            since = f"since attach at {self._attached_iso()} ({elapsed} s ago)"
        else:
            since = "and no attach has happened"
        return (
            f"no message received on {topic} {since}; "
            f"received={entry.get('received', 0)}, errors={entry.get('errors', 0)}"
        )

    def _log(self, message: str) -> None:
        if self.logger is not None:
            try:
                self.logger.info(message)
            except Exception:
                pass


# The camera's declared extrinsics, mirroring
# sim_moveit_bringup/config/cameras.yaml's single `camera_depth` entry. Recorded
# here as data rather than re-parsed from that file because this process has no
# launch context and the yaml lives in another package's share directory; the
# static TF broadcaster is the thing that actually publishes it. If the yaml
# changes, this must change with it -- it is reported to the model as the camera
# pose, so a silent divergence would be a lie in the record.
CAMERA_EXTRINSICS_FROM_YAML: dict[str, Any] = {
    "parent": "world",
    "child": "camera_depth_optical_frame",
    "xyz": [0.25, -0.2, 1.7],
    "quat_xyzw": [0.0, 0.99863, -0.05234, 0.0],
    "source": "sim_moveit_bringup/config/cameras.yaml",
}

# Byte layout of the raw colour encodings this cell publishes. Mirrors
# utils/png_encode._CHANNELS, but this map additionally records WHERE the red
# channel is, which png_encode handles by swapping bytes and this module handles
# by indexing.
_RGB_LAYOUTS = {
    "rgb8": (3, (0, 1, 2)),
    "bgr8": (3, (2, 1, 0)),
    "rgba8": (4, (0, 1, 2)),
    "bgra8": (4, (2, 1, 0)),
}


def rgb_to_array(data: bytes, width: int, height: int, encoding: str) -> Any | None:
    """HxWx3 uint8 RGB from a raw sensor_msgs/Image buffer, or None.

    Alpha is dropped rather than composited: nothing downstream uses it, and a
    look() view is for a human-or-model to read, not to blend. Returns None for
    an encoding this cell cannot interpret or a buffer too short for its stated
    dimensions -- callers report that as a failed view rather than showing the
    model a misaligned image, which is the kind of wrong that is easy to miss.
    """
    import numpy as np

    enc = (encoding or "").strip().lower()
    layout = _RGB_LAYOUTS.get(enc)
    if layout is None or width <= 0 or height <= 0:
        return None
    channels, order = layout
    expected = width * height * channels
    if len(data) < expected:
        return None
    flat = np.frombuffer(data, dtype=np.uint8, count=expected)
    image = flat.reshape(height, width, channels)
    return np.ascontiguousarray(image[:, :, list(order)])


def depth_image_to_metres(msg: Any) -> Any | None:
    """HxW float32 metres from a 32FC1 (or 16UC1 millimetre) depth Image.

    32FC1 is what the Isaac render product publishes and is already metres.
    16UC1 is accepted because it is the other convention a depth publisher may
    be configured for, and reading it as metres without the /1000 would be
    wrong by three orders of magnitude -- a failure that would look like a
    plausible scene rather than an error. Anything else returns None.
    """
    import numpy as np

    enc = str(getattr(msg, "encoding", "") or "").strip().lower()
    width = int(getattr(msg, "width", 0) or 0)
    height = int(getattr(msg, "height", 0) or 0)
    data = bytes(getattr(msg, "data", b"") or b"")
    if width <= 0 or height <= 0:
        return None
    count = width * height
    if enc == "32fc1":
        if len(data) < count * 4:
            return None
        return np.frombuffer(data, dtype="<f4", count=count).reshape(height, width).copy()
    if enc == "16uc1":
        if len(data) < count * 2:
            return None
        raw = np.frombuffer(data, dtype="<u2", count=count).reshape(height, width)
        metres = raw.astype(np.float32) / 1000.0
        # 0 is the standard "no return" sentinel in 16UC1 depth, and leaving it
        # as a literal 0 m would place a surface at the lens.
        metres[raw == 0] = np.nan
        return metres
    return None


def camera_info_to_intrinsics(msg: Any) -> dict[str, Any] | None:
    """{fx, fy, cx, cy, width, height, K, D, distortion_model} or None.

    Returns None when K is absent or fx/fy are zero -- an unset CameraInfo
    publishes all-zero K, and dividing by it would produce a projection full of
    infinities rather than an obvious failure.
    """
    try:
        # No `k or K`: rosidl exposes CameraInfo.k as a numpy array, whose
        # truth value raises -- which the except below would silently turn
        # into "no intrinsics" for every real message.
        k_raw = getattr(msg, "k", None)
        if k_raw is None:
            k_raw = getattr(msg, "K", None)
        k = [float(v) for v in (k_raw if k_raw is not None else [])]
        width = int(getattr(msg, "width", 0) or 0)
        height = int(getattr(msg, "height", 0) or 0)
    except Exception:
        return None
    if len(k) < 9 or width <= 0 or height <= 0:
        return None
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx == 0.0 or fy == 0.0:
        return None
    try:
        d_raw = getattr(msg, "d", None)
        if d_raw is None:
            d_raw = getattr(msg, "D", None)
        d = [float(v) for v in (d_raw if d_raw is not None else [])]
    except Exception:
        d = []
    return {
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "width": width, "height": height,
        "K": k[:9], "D": d,
        "distortion_model": str(getattr(msg, "distortion_model", "") or ""),
    }


def cloud_xyz(msg: Any) -> Any | None:
    """Nx3 float32 xyz from a PointCloud2, or None.

    Parsed straight out of the message bytes with a numpy structured dtype
    built from msg.fields, because this cell has no ros2_numpy or
    sensor_msgs_py dependency on the vision path and adding one for three
    field offsets would not earn its place. Handles arbitrary field order and
    padding by reading point_step-sized records and taking the x/y/z offsets
    the message itself declares.
    """
    import numpy as np

    try:
        point_step = int(msg.point_step)
        fields = {f.name: f for f in msg.fields}
        data = bytes(msg.data)
    except Exception:
        return None
    if point_step <= 0 or not data:
        return None
    if not all(axis in fields for axis in ("x", "y", "z")):
        return None
    try:
        spec = []
        for axis in ("x", "y", "z"):
            field = fields[axis]
            dtype = _POINTFIELD_DTYPES.get(int(field.datatype))
            if dtype is None:
                return None
            spec.append((axis, dtype, int(field.offset)))
    except Exception:
        return None

    n = len(data) // point_step
    if n <= 0:
        return None
    structured = np.dtype({
        "names": [s[0] for s in spec],
        "formats": [s[1] for s in spec],
        "offsets": [s[2] for s in spec],
        "itemsize": point_step,
    })
    records = np.frombuffer(data, dtype=structured, count=n)
    xyz = np.empty((n, 3), dtype=np.float32)
    for index, axis in enumerate(("x", "y", "z")):
        xyz[:, index] = records[axis].astype(np.float32)
    return xyz


def project_cloud_to_depth(msg: Any, intrinsics: dict[str, Any]) -> Any | None:
    """HxW float32 depth from a PointCloud2, NaN where no point landed.

    The cloud is already expressed in the camera's own optical frame (it is
    published on the same render product as the colour image), so this is a
    pinhole projection and no TF lookup is involved:
        u = fx * x / z + cx,  v = fy * y / z + cy

    Where several points land on one pixel the NEAREST is kept, which is what a
    depth image means -- a surface occludes what is behind it, and keeping the
    farther point would let the background show through the object. Implemented
    with np.minimum.at over a flat index, so it is one pass and order-independent.

    The result is SPARSE. A projected cloud is not a substitute for a depth
    image: the cloud may already be decimated or workspace-filtered, and every
    pixel no point reached stays NaN. Callers must report the source so the model
    is never left to assume it is looking at dense per-pixel depth.
    """
    import numpy as np

    xyz = cloud_xyz(msg)
    if xyz is None or xyz.shape[0] == 0:
        return None
    width = int(intrinsics["width"])
    height = int(intrinsics["height"])
    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    # Points at or behind the lens cannot project, and NaN/inf coordinates are
    # normal in an organised cloud's empty cells.
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > 0.0)
    if not valid.any():
        return None
    x, y, z = x[valid], y[valid], z[valid]

    u = np.rint(fx * x / z + cx).astype(np.int64)
    v = np.rint(fy * y / z + cy).astype(np.int64)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    if not inside.any():
        return None
    u, v, z = u[inside], v[inside], z[inside]

    depth = np.full(width * height, np.inf, dtype=np.float32)
    np.minimum.at(depth, v * width + u, z)
    depth[np.isinf(depth)] = np.nan
    return depth.reshape(height, width)


__all__ = [
    "ImageFrame",
    "ImageRingBuffer",
    "ImageBufferManager",
    "CameraStreams",
    "CAMERA_EXTRINSICS_FROM_YAML",
    "DEFAULT_STALE_SEC",
    "camera_info_to_intrinsics",
    "cloud_xyz",
    "depth_image_to_metres",
    "project_cloud_to_depth",
    "rgb_to_array",
]
