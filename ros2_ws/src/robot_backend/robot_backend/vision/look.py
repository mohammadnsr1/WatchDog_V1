"""look(): return current camera frames to the model as image content.

This is the one place the two servers' look() tools share, so the WatchDog and
the executor are looking through the same instrument. See the package docstring
for why that matters.

What look() is NOT: it is not capture_frames. capture_frames reaches BACKWARDS
into a ring buffer to preserve a moment that has already passed, writes PNGs, and
returns paths -- the agent never sees those pixels. look() reads the LATEST frame
of each stream and returns it to the model as an MCP image content block, so the
model can actually see it. Both write to <session_dir>/frames/ and both appear on
the event timeline, and telling them apart in the record is the point of having
two event types.

Failure stance: never raise to the model. A camera that is not publishing, a
depth topic that does not exist yet, a stale stream, an encoding this cell cannot
read -- each of those degrades a view and is reported in the text part by name
and reason. An agent that asked to look and got nothing must be told why it got
nothing, because "I looked and saw nothing" and "I could not look" lead to
opposite conclusions about the workspace.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from robot_backend.utils.png_encode import encode_png
from robot_backend.watchdog.image_buffer import DEFAULT_STALE_SEC
from robot_backend.watchdog.timebase import wall_ns

# Views look() knows how to render. Kept as a frozen set so an unknown view is
# rejected with the list of real ones rather than silently producing nothing.
VALID_VIEWS = ("rgb", "depth")

# Percentile clip for the depth visualisation. The raw range of a depth frame is
# dominated by whatever is furthest away (the floor, or a NaN-adjacent outlier),
# so a linear map over min..max renders the whole workspace in two or three
# indistinguishable grey levels. Clipping to the 2nd/98th percentile of the
# VALID pixels spends the contrast on the scene instead. The chosen range is
# reported to the model with every depth view -- a visualisation whose mapping is
# unstated is not evidence, and the agent must be able to say what brightness
# meant when it reasons from one.
DEPTH_CLIP_PERCENTILES = (2.0, 98.0)

# Per-process look counter, for the saved filenames. Not a global run counter:
# the WatchDog and the executor are separate processes with separate sequences,
# which is why the filename prefix differs per role and why look_id also carries
# a random suffix.
_SEQ_LOCK = threading.Lock()
_SEQ = 0


def _next_seq() -> int:
    global _SEQ
    with _SEQ_LOCK:
        _SEQ += 1
        return _SEQ


@dataclass
class LookDeps:
    """Everything look() needs from whichever server is hosting it.

    streams           -- a CameraStreams; the only source of pixels.
    session_dir       -- run directory; frames land in <session_dir>/frames/.
    role              -- "watchdog" or "executor", onto the event and the
                         filename prefix.
    file_prefix       -- "look_" or "exec_look_", so the two agents' frames
                         cannot collide in the shared frames directory.
    publish_event     -- called with one EventEnvelope, or None to skip
                         publishing (unit tests, and any host with no bus).
    current_step_id   -- callable returning the step in progress, or None. A
                         look between steps legitimately has no step.
    logger            -- optional ROS logger for non-fatal complaints.
    """

    streams: Any
    session_dir: Path
    role: str
    file_prefix: str
    publish_event: Callable[[Any], None] | None = None
    current_step_id: Callable[[], str | None] | None = None
    logger: Any | None = None

    @property
    def stale_sec(self) -> float:
        """The staleness threshold, read from the streams that enforce it.

        Deliberately NOT a field. CameraStreams is what actually decides a view
        is stale; a second copy here could disagree with it, and the only thing
        this value is used for is telling the agent which threshold was applied.
        A reported threshold that is not the enforced one is worse than none.
        """
        return float(getattr(self.streams, "stale_sec", DEFAULT_STALE_SEC))


@dataclass
class LookResult:
    """One look's outcome, before it is turned into MCP content.

    Separated from the content-building step so the whole decision path is
    testable without FastMCP installed, and so the event payload and the text
    part are built from ONE set of facts rather than assembled twice.
    """

    look_id: str
    role: str
    reason: str
    views_requested: list[str]
    wall_ns: int
    views_ok: list[str] = field(default_factory=list)
    views_failed: list[dict[str, Any]] = field(default_factory=list)
    paths: dict[str, str] = field(default_factory=dict)
    stamps_ns: dict[str, int] = field(default_factory=dict)
    ages_ms: dict[str, float] = field(default_factory=dict)
    # view -> (png_bytes, width, height) at the DOWNSCALED size that goes to
    # the model. The full-resolution version is what lands on disk.
    images: dict[str, tuple[bytes, int, int]] = field(default_factory=dict)
    depth_source: str | None = None
    depth_clip_m: list[float] | None = None
    image_size: list[int] | None = None
    camera_frame_id: str | None = None
    camera: dict[str, Any] | None = None
    step_id: str | None = None
    notes: list[str] = field(default_factory=list)
    # CameraStreams.stats(): per-stream topic, receive counts, last-receive
    # time and error counts, so a look whose views failed carries the receive
    # accounting that explains why.
    streams: dict[str, Any] | None = None

    def text_payload(self) -> dict[str, Any]:
        """The JSON object that becomes look()'s text content part."""
        return {
            "ok": bool(self.views_ok),
            "command": "look",
            "look_id": self.look_id,
            "role": self.role,
            "reason": self.reason,
            "wall_ns": self.wall_ns,
            "step_id": self.step_id,
            "views_requested": self.views_requested,
            "views_ok": self.views_ok,
            "views_failed": self.views_failed,
            "stamps_ns": self.stamps_ns,
            "ages_ms": self.ages_ms,
            "image_size": self.image_size,
            "returned_size": {
                view: {"width": w, "height": h}
                for view, (_, w, h) in self.images.items()
            },
            "depth_source": self.depth_source,
            "depth_clip_m": self.depth_clip_m,
            "depth_clip_note": (
                "Depth is rendered grayscale with NEAR = BRIGHT, linearly mapped "
                f"over {self.depth_clip_m} metres (the "
                f"{DEPTH_CLIP_PERCENTILES[0]:g}th/{DEPTH_CLIP_PERCENTILES[1]:g}th "
                "percentiles of the valid pixels). Black is either NaN (no depth "
                "at that pixel) or at/beyond the far clip. Brightness is only "
                "comparable within this one image."
                if self.depth_clip_m is not None else None
            ),
            "camera_frame_id": self.camera_frame_id,
            "camera": self.camera,
            # The full receive accounting earns its size only when something
            # failed; a clean look carries a one-line health summary instead.
            "streams": self.streams if self.views_failed else None,
            "streams_ok": None if self.views_failed else self._streams_ok(),
            "paths": self.paths,
            "notes": self.notes,
        }

    def _streams_ok(self) -> dict[str, Any] | None:
        """One-line stream health for a look with no failed view."""
        if not self.streams:
            return None
        per_stream = self.streams.get("streams") or {}
        attached = self.streams.get("attached_wall_ns")
        elapsed_sec = (
            max((self.wall_ns - attached) / 1e9, 1e-9)
            if attached is not None else None
        )

        def hz(name: str) -> float | None:
            received = (per_stream.get(name) or {}).get("received") or 0
            if not received or elapsed_sec is None:
                return None
            return round(received / elapsed_sec, 2)

        return {
            "rgb_hz_est": hz("rgb"),
            "depth_hz_est": hz("depth"),
            "camera_info_topic": (per_stream.get("camera_info") or {}).get("topic"),
        }


def downscale(array: Any, max_width: int) -> Any:
    """Decimate an HxW or HxWx3 array so width <= max_width, keeping aspect.

    Integer stride, not interpolation. The factor is ceil(width / max_width), so
    the aspect ratio is preserved exactly and both axes are reduced by the same
    integer -- no resampling, no new pixel values, and nothing that could invent
    a feature that was not in the frame. For depth in particular, averaging
    neighbouring pixels would produce distances at which no surface exists,
    which is worse than a slightly coarser image.
    """
    import numpy as np

    if max_width <= 0:
        return array
    height, width = int(array.shape[0]), int(array.shape[1])
    if width <= max_width:
        return array
    factor = int(np.ceil(width / float(max_width)))
    if factor <= 1:
        return array
    return np.ascontiguousarray(array[::factor, ::factor])


def depth_to_grayscale_png(depth: Any) -> tuple[bytes, list[float]] | None:
    """(mono8 PNG bytes, [clip_lo, clip_hi]) for a float depth array, or None.

    Near = bright, so the visualisation reads the way a person expects a
    proximity image to read. Mapping:
      - valid pixels are the finite ones; NaN is "no depth here", not "far".
      - the range is the [p2, p98] percentiles of the valid pixels, so a single
        outlier cannot flatten the whole image (see DEPTH_CLIP_PERCENTILES).
      - value = 255 * (1 - (d - lo) / (hi - lo)), clipped to 0..255.
      - NaN renders BLACK, which deliberately collides with "at or beyond the
        far clip". That ambiguity is why the clip range and this note travel
        with every depth view: the agent must not read black as "far away" when
        it may mean "nothing was measured".

    Returns None when no pixel is valid -- an all-NaN depth frame is a failed
    view, not a black image, and showing the model a black square would invite
    it to conclude the workspace is empty.
    """
    import numpy as np

    array = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(array)
    if not valid.any():
        return None
    values = array[valid]
    lo = float(np.percentile(values, DEPTH_CLIP_PERCENTILES[0]))
    hi = float(np.percentile(values, DEPTH_CLIP_PERCENTILES[1]))
    if not (hi > lo):
        # A genuinely flat depth frame (every valid pixel at one distance).
        # Widen by a millimetre so the division is defined; the image will be
        # uniform, which is the truth about that frame.
        hi = lo + 1e-3
    normalised = (array - lo) / (hi - lo)
    grey = np.clip((1.0 - normalised) * 255.0, 0.0, 255.0)
    grey[~valid] = 0.0
    buffer = grey.astype(np.uint8)
    height, width = buffer.shape[0], buffer.shape[1]
    png = encode_png(buffer.tobytes(), width, height, "mono8")
    return png, [round(lo, 6), round(hi, 6)]


def _save(path: Path, data: bytes, notes: list[str], logger: Any | None) -> str | None:
    """Write bytes, returning the path as a string, or None after complaining.

    A failed save never fails the look: the model has already been shown the
    frame in the response, and losing the on-disk copy costs the audit trail,
    not the observation. It is recorded in notes so the gap is visible rather
    than silent.
    """
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return str(path)
    except Exception as exc:
        notes.append(f"could not save {path.name}: {type(exc).__name__}: {exc}")
        if logger is not None:
            try:
                logger.warning(f"look: save failed for {path}: {exc}")
            except Exception:
                pass
        return None


def _normalise_views(views: Any) -> tuple[list[str], list[dict[str, Any]]]:
    """Requested views, de-duplicated and order-preserving, plus rejections."""
    if views is None:
        return ["rgb"], []
    if isinstance(views, str):
        views = [views]
    ok: list[str] = []
    failed: list[dict[str, Any]] = []
    try:
        iterator = list(views)
    except TypeError:
        return [], [{"view": repr(views), "reason": "views must be a list of strings"}]
    for entry in iterator:
        name = str(entry).strip().lower()
        if name not in VALID_VIEWS:
            failed.append({
                "view": str(entry),
                "reason": f"unknown view; valid views are {list(VALID_VIEWS)}",
            })
            continue
        if name not in ok:
            ok.append(name)
    return ok, failed


# A look issued moments after the camera subscriptions attached may simply be
# ahead of the first frame (the live executor's first look was 0 s after
# attach). Within this bound after attach, a view whose slot has NEVER
# received waits for its first message rather than failing. A slot that HAS
# received never waits: stale is stale.
FIRST_FRAME_GRACE_SEC = 3.0


def _wait_for_first_frames(streams: Any, requested: list[str]) -> str | None:
    """Poll (50 ms) until every never-received requested slot fills, bounded
    to FIRST_FRAME_GRACE_SEC after attach. Returns a note, or None if no wait
    applied."""
    view_streams = {"rgb": "rgb", "depth": "depth"}
    wanted = [view_streams[v] for v in requested if v in view_streams]
    try:
        attached = streams.attached_wall_ns
        if attached is None or not wanted:
            return None
        per_stream = streams.stats()["streams"]
        pending = [s for s in wanted if per_stream[s]["received"] == 0]
    except Exception:
        return None
    if not pending:
        return None
    deadline = int(attached) + int(FIRST_FRAME_GRACE_SEC * 1e9)
    start = wall_ns()
    if start >= deadline:
        return None
    waited = False
    while wall_ns() < deadline:
        try:
            per_stream = streams.stats()["streams"]
            if all(per_stream[s]["received"] > 0 for s in pending):
                break
        except Exception:
            break
        waited = True
        time.sleep(0.05)
    if not waited:
        return None
    return (
        f"waited {round((wall_ns() - start) / 1e6)} ms for first frame after attach"
    )


def perform_look(
    deps: LookDeps,
    reason: str,
    views: Any = None,
    max_width: int = 640,
) -> LookResult:
    """Read the requested views, save them, and return the decided result.

    Does everything except build MCP content and publish the event, so the whole
    path is testable without FastMCP or ROS. build_look_content() and
    build_look_event() take it from here.
    """
    import numpy as np

    requested, failed = _normalise_views(views)
    # A first look right after attach beats the first frame; wait it out
    # (bounded) instead of failing views the camera is about to serve.
    wait_note = _wait_for_first_frames(deps.streams, requested)
    now = wall_ns()
    seq = _next_seq()
    result = LookResult(
        look_id=f"{seq:04d}-{uuid.uuid4().hex[:8]}",
        role=deps.role,
        reason=str(reason),
        views_requested=requested,
        wall_ns=now,
        views_failed=failed,
    )
    if wait_note:
        result.notes.append(wait_note)
    if deps.current_step_id is not None:
        try:
            result.step_id = deps.current_step_id()
        except Exception:
            result.step_id = None

    frames_dir = Path(deps.session_dir) / "frames"
    ages = {}
    try:
        ages = deps.streams.ages()
    except Exception as exc:
        result.notes.append(f"could not read stream ages: {exc}")
    try:
        result.streams = deps.streams.stats()
    except Exception as exc:
        result.notes.append(f"could not read stream stats: {exc}")

    # Camera metadata is reported whether or not any view succeeded: knowing the
    # intrinsics are missing is exactly what explains a missing depth fallback.
    try:
        camera = deps.streams.get_camera()
    except Exception as exc:
        camera = None
        result.notes.append(f"camera_info unavailable: {exc}")
    if camera is not None:
        result.camera = camera
        result.camera_frame_id = camera.get("frame_id")
    else:
        result.notes.append(
            "camera_info is not available, so intrinsics and the camera frame_id "
            "are unknown and the point-cloud depth fallback cannot be projected."
        )

    for view in requested:
        if view == "rgb":
            age = ages.get("rgb")
            try:
                got = deps.streams.get_rgb()
            except Exception as exc:
                got = None
                result.views_failed.append(
                    {"view": "rgb", "reason": f"{type(exc).__name__}: {exc}"}
                )
                continue
            if got is None:
                if age is None:
                    try:
                        reason = deps.streams.never_received_text("rgb")
                    except Exception:
                        reason = "no RGB frame available"
                else:
                    reason = (
                        f"latest RGB frame is {age:.2f}s old "
                        f"(stale > {deps.stale_sec}s), or its encoding could not be read"
                    )
                result.views_failed.append({
                    "view": "rgb", "reason": reason, "age_sec": age,
                })
                continue
            array, stamp_ns = got
            height, width = int(array.shape[0]), int(array.shape[1])
            result.image_size = [width, height]
            result.stamps_ns["rgb"] = int(stamp_ns)
            if age is not None:
                result.ages_ms["rgb"] = round(age * 1000.0, 3)

            full = encode_png(
                np.ascontiguousarray(array).tobytes(), width, height, "rgb8"
            )
            saved = _save(
                frames_dir / f"{deps.file_prefix}{seq:04d}_rgb.png",
                full, result.notes, deps.logger,
            )
            if saved:
                result.paths["rgb"] = saved

            small = downscale(array, int(max_width))
            s_h, s_w = int(small.shape[0]), int(small.shape[1])
            result.images["rgb"] = (
                encode_png(np.ascontiguousarray(small).tobytes(), s_w, s_h, "rgb8"),
                s_w, s_h,
            )
            result.views_ok.append("rgb")

        elif view == "depth":
            try:
                got = deps.streams.get_depth()
            except Exception as exc:
                got = None
                result.views_failed.append(
                    {"view": "depth", "reason": f"{type(exc).__name__}: {exc}"}
                )
                continue
            if got is None:
                try:
                    why = deps.streams.describe_depth_availability()
                except Exception as exc:
                    why = f"depth unavailable and the reason could not be read: {exc}"
                result.views_failed.append({
                    "view": "depth",
                    "reason": why,
                    "age_sec": ages.get("depth"),
                    "pointcloud_age_sec": ages.get("pointcloud"),
                })
                continue
            depth_array, stamp_ns, source = got
            result.depth_source = source
            result.stamps_ns["depth"] = int(stamp_ns)
            if result.image_size is None:
                # Set here too, not only in the rgb branch: a depth-only look
                # would otherwise report image_size: null while having returned
                # a perfectly good image. RGB wins when both ran, since the two
                # come off one render product and agree.
                result.image_size = [
                    int(depth_array.shape[1]), int(depth_array.shape[0])
                ]
            age = ages.get("depth") if source == "depth_image" else ages.get("pointcloud")
            if age is not None:
                result.ages_ms["depth"] = round(age * 1000.0, 3)

            # The raw metric array is saved BEFORE any visualisation, so a later
            # audit can recompute what the agent should have been able to see
            # rather than having to trust the grayscale mapping.
            npy_saved = _save_npy(
                frames_dir / f"{deps.file_prefix}{seq:04d}_depth.npy",
                depth_array, result.notes, deps.logger,
            )
            if npy_saved:
                result.paths["depth_npy"] = npy_saved

            full_png = depth_to_grayscale_png(depth_array)
            if full_png is None:
                result.views_failed.append({
                    "view": "depth",
                    "reason": (
                        f"depth arrived from {source} but no pixel held a finite "
                        "distance, so there is nothing to show"
                    ),
                })
                continue
            full_bytes, clip = full_png
            result.depth_clip_m = clip
            saved = _save(
                frames_dir / f"{deps.file_prefix}{seq:04d}_depth.png",
                full_bytes, result.notes, deps.logger,
            )
            if saved:
                result.paths["depth"] = saved

            small = downscale(depth_array, int(max_width))
            small_png = depth_to_grayscale_png(small)
            if small_png is None:
                # Decimation dropped every valid pixel (a very sparse projected
                # cloud). Fall back to the full-resolution rendering rather than
                # report a view that did succeed as failed.
                d_h, d_w = int(depth_array.shape[0]), int(depth_array.shape[1])
                result.images["depth"] = (full_bytes, d_w, d_h)
                result.notes.append(
                    "depth was returned at full resolution: downscaling left no "
                    "valid pixels, which happens when a projected cloud is sparse."
                )
            else:
                small_bytes, small_clip = small_png
                result.depth_clip_m = small_clip
                result.images["depth"] = (
                    small_bytes, int(small.shape[1]), int(small.shape[0])
                )
            result.views_ok.append("depth")

    if not result.views_ok:
        result.notes.append(
            "No view could be returned. This says nothing about the workspace -- "
            "it says the camera streams were unavailable. Do not read it as an "
            "empty or unchanged scene."
        )
    return result


def _save_npy(
    path: Path, array: Any, notes: list[str], logger: Any | None
) -> str | None:
    """Write a float32 .npy beside the depth PNG. Failure is non-fatal."""
    import numpy as np

    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as handle:
            np.save(handle, np.asarray(array, dtype=np.float32), allow_pickle=False)
        return str(path)
    except Exception as exc:
        notes.append(f"could not save {path.name}: {type(exc).__name__}: {exc}")
        if logger is not None:
            try:
                logger.warning(f"look: npy save failed for {path}: {exc}")
            except Exception:
                pass
        return None


def build_look_content(result: LookResult) -> list[Any]:
    """The MCP content list for a look: one text part, then one part per view.

    The text part is a JSON string rather than a dict so its serialisation is
    ours and not FastMCP's -- which matters because the tool is registered with
    structured_output=False (there is no output schema to validate against) and
    because a dict in this list would be JSON-dumped by _convert_to_content
    anyway, just with formatting we do not control.

    FastMCP's _convert_to_content flattens a list and turns each Image into an
    ImageContent block, so this list is exactly what reaches the model: the
    caption, then the pictures.
    """
    from mcp.server.fastmcp import Image

    parts: list[Any] = [json.dumps(result.text_payload(), indent=2)]
    for view in result.views_ok:
        entry = result.images.get(view)
        if entry is None:
            continue
        png_bytes, _, _ = entry
        parts.append(Image(data=png_bytes, format="png"))
    return parts


def build_look_event(result: LookResult) -> Any:
    """One "look" EventEnvelope from a LookResult. Paths only, never bytes."""
    from robot_backend.events.schema import LookPayload, make_event

    payload = LookPayload(
        look_id=result.look_id,
        role=result.role,
        reason=result.reason,
        views_requested=result.views_requested,
        views_ok=result.views_ok,
        views_failed=result.views_failed,
        paths=result.paths,
        stamps_ns=result.stamps_ns,
        ages_ms=result.ages_ms,
        wall_ns=result.wall_ns,
        depth_source=result.depth_source,
        depth_clip_m=result.depth_clip_m,
        image_size=result.image_size,
        camera_frame_id=result.camera_frame_id,
        step_id=result.step_id,
        notes=list(result.notes),
    )
    return make_event("look", payload)


def _tool_failure(role: str, exc: BaseException, stage: str) -> list[Any]:
    """The one shape a look failure takes when it reaches the model."""
    return [json.dumps({
        "ok": False,
        "command": "look",
        "role": role,
        "stage": stage,
        "error": f"{type(exc).__name__}: {exc}",
        "note": (
            "look() failed and returned no frames. This is a TOOL failure, not an "
            "observation: draw no conclusion about the workspace from it."
        ),
    }, indent=2)]


def look(
    deps: LookDeps | Callable[[], LookDeps],
    reason: str,
    views: Any = None,
    max_width: int = 640,
    role: str = "unknown",
) -> list[Any]:
    """Run a look end to end: read, save, publish the event, return content.

    This is the whole body of both servers' look() tools, and it never raises. A
    tool that throws leaves the agent with no information at all about why it
    cannot see, which is the one outcome that must not happen: "I looked and saw
    nothing" and "I could not look" lead to opposite conclusions.

    deps may be a LookDeps or a zero-argument callable returning one. The
    callable form exists so a host whose setup can fail -- lazily creating a ROS
    node, resolving the session directory, configuring the event bus -- can do
    that work INSIDE this guard rather than before it, where an exception would
    escape to the model as a raw tool error. role is used only to label such a
    failure, since deps may not exist yet when it happens.
    """
    if callable(deps):
        try:
            deps = deps()
        except Exception as exc:
            return _tool_failure(role, exc, "setup")
    role = deps.role

    try:
        result = perform_look(deps, reason, views, max_width)
    except Exception as exc:
        return _tool_failure(role, exc, "capture")

    if deps.publish_event is not None:
        try:
            deps.publish_event(build_look_event(result))
        except Exception as exc:
            result.notes.append(f"look event not published: {type(exc).__name__}: {exc}")
            if deps.logger is not None:
                try:
                    deps.logger.warning(f"look: event publish failed: {exc}")
                except Exception:
                    pass

    try:
        return build_look_content(result)
    except Exception as exc:
        # FastMCP unimportable, or Image construction failed. The text part
        # still carries every path and stamp, so the look is not lost.
        return [json.dumps({
            **result.text_payload(),
            "image_content_error": f"{type(exc).__name__}: {exc}",
        }, indent=2)]


__all__ = [
    "DEPTH_CLIP_PERCENTILES",
    "LookDeps",
    "LookResult",
    "VALID_VIEWS",
    "build_look_content",
    "build_look_event",
    "depth_to_grayscale_png",
    "downscale",
    "look",
    "perform_look",
]
