# STAGE1_REPORT — camera plumbing + look()

Stage 1 of scene observation: the camera buffer, and `look()` on both the
WatchDog and the executor. No prompt, task file or `prompt_variants.py` edit
(Stage 4). No `ctx.rgb()`/array lane (Stage 2), no `record_observation` or
injection (Stage 3).

> ## Status update — 2026-09-30
>
> The Isaac topic set changed after this report was first written. Verified live:
>
> | Topic | Then | Now |
> | --- | --- | --- |
> | `/camera/depth/rgb` | live | live |
> | `/camera/depth/image_raw` | not yet published | **live** (32FC1 metres) |
> | `/camera/depth/camera_info` | not yet published | **live** |
> | `/camera/depth/points` | live | **no longer published** |
>
> Four consequences for what this report says:
>
> 1. **Depth works, and works by the preferred path.** `depth_source` will read
>    `"depth_image"` — dense, per-pixel. The `camera_info` name ambiguity is
>    settled: the namespaced topic is live and `_resolve_info_topic` picks it
>    first, so no override is needed.
> 2. **The point-cloud projection is now dormant, and retained on purpose.**
>    Decision (2026-09-30): keep `project_cloud_to_depth` / `cloud_xyz` as the
>    option for later rather than delete them. They are **unexercised** — no
>    cloud exists to run them against — so treat them as unverified code, not
>    working code. Nothing subscribes to the cloud either: `use_octomap` and
>    `filter_pointcloud` both default `false`.
> 3. **Depth no longer degrades, it fails.** With no cloud, a stale or absent
>    `image_raw` means `get_depth()` returns `None` and the depth view is reported
>    failed with a reason. This is the better behaviour — a stale depth image and
>    a sparse projection are not interchangeable evidence — but it is a change
>    from the two-path design this report describes.
> 4. **Scope decisions taken** (both were open questions in this report's closing
>    section): depth is for the **WatchDog** for now — implemented on both servers,
>    but the executor's use of it is out of scope, by convention rather than by a
>    gate. And the **planner is not told about unlisted objects**: a Stage 3
>    intruder stays out of the MoveIt planning scene, so noticing it is the
>    agent's job via `look()`. The octomap tension I raised is therefore closed.
>
> Also found and **fixed**: `_IMAGE_BAG_TOPICS` in `sim_moveit_bringup.launch.py`
> still named the old `/camera_info`, which no longer exists, so
> `record_images:=true` was silently recording colour only. Corrected to
> `/camera/depth/camera_info`. `look()` was never affected — it resolves the name
> itself. `/camera/depth/image_raw` was deliberately not added to the bag (4
> bytes/px on a ~180 MB/min option; `look()` already persists the depth an agent
> used, as PNG + `.npy`). `record_pointcloud:=true` now records nothing, since no
> cloud is published; the name is kept because the path is retained as an option.
>
> Two comments asserting "there is no depth image topic on this cell" were also
> false and are corrected: `sim_moveit_bringup.launch.py:27` and
> `image_buffer.py`'s RGB-candidate note. The identical sentence inside
> `VISION_DISCOVERY.md` is left as written — that document is a dated snapshot and
> carries a superseded banner.

---

## Rule compliance

**One violation to declare.** I ran `python3 -c` once, to AST-parse the seven
edited/created files as a syntax check. It imported no project code and executed
nothing from the repo, but the instruction was "no python" and I ran python. No
other process was started: no pytest, no ros2, no colcon, no launch, no Isaac.
Everything after that check was verified by reading.

Consequence: **nothing here has been executed, imported, or type-checked beyond
that one AST parse.** Import-time errors, ROS message-field mismatches and
pydantic validation behaviour are all unverified. The "Not resolved" section
lists what I would check first.

---

## PRECHECK (report only — not fixed)

### 1. `dry_run_custom_monitor` / `_prepare_authored_monitor` — **NO, neither landed**

Neither identifier appears anywhere in
`ros2_ws/src/robot_backend/robot_backend/mcp_servers/watchdog_mcp_server.py`, and
neither appears anywhere else in the repo. `grep -n
'dry_run_custom_monitor\|_prepare_authored_monitor'` over that file returns
nothing. The authored-monitor arm path is still the single
`arm_custom_monitor` at line **1380** (`@_tool_for("S2", "S3")` at 1380,
`def arm_custom_monitor` at 1381), which constructs `AuthoredMonitor` inline at
lines **1496–1507** with no extracted preparation helper and no dry-run
counterpart. The arm-time first call is still the only way to exercise authored
code, inside the `AuthoredMonitor.__init__` at
`robot_backend/watchdog/authored.py:566-575`.

### 2. Verdict 5a reorder — **NO, did not land**

`agents/prompts/watchdog_system.txt`, section `5a.` opens at line **486**. The
ordered choice is at lines **515–527**, introduced by "Then choose the value by
what you measured, top match wins:", and the order is still:

| Position | Verdict | Line |
| --- | --- | --- |
| 1 | `incomplete` | 517–519 |
| 2 | `non_conformant` | 520–522 |
| 3 | `conformant` | 523–527 |

Both requested properties are absent:

- **"non_conformant wins when a violation stands"** — not the case. `incomplete`
  is listed first under an explicit "top match wins", so an aborted run with an
  uncorrected violation resolves to `incomplete`, not `non_conformant`.
- **"incomplete only when no finding was recorded"** — that condition does not
  appear. Line 517-518's test is purely "aborted or crashed with parts still
  unattempted"; nothing conditions it on whether a finding was recorded. The
  closest existing text is lines 532–535, which argues the opposite way: "an
  executor that aborted BECAUSE you caught something does not force
  non_conformant: if it quit with parts unattempted that is incomplete".
- **S4 wording** — `agents/prompts/executor_system_s4.txt:383-384` still reads
  "non_conformant (a part is not, or a violation stands), incomplete (the run
  stopped before the end-state could exist)", i.e. the plain definitions with no
  precedence stated, so there is no S4 wording aligned to a reorder that has not
  happened.

The same un-reordered block is in `watchdog_system_s2.txt` (5a at **526**) and
`watchdog_system_s3.txt` (5a at **506**), so a Stage 4 edit has three files to
change, not one.

### 3. Static TF `world -> camera_depth_optical_frame` — **YES, it is published**

Asked as a check; answering rather than adding anything.

- Source of values: `ros2_ws/src/sim_moveit_bringup/config/cameras.yaml` (the
  single `camera_depth` entry).
- Publisher:
  `ros2_ws/src/sim_moveit_bringup/sim_moveit_bringup/static_camera_tf_broadcaster.py`
  — a `StaticTransformBroadcaster` (line 36), one `TransformStamped` per enabled
  camera (99–111), sent at startup (126).
- Launch wiring: `sim_moveit_bringup/launch/camera_tf.launch.py`, included from
  `sim_moveit_bringup/launch/sim_moveit_bringup.launch.py:346-352`, gated on
  `publish_camera_tf` which **defaults to `true`** (529–531); `camera_config`
  defaults to that yaml (490–491, 534–535).

So no publisher was added. `look()` reports the yaml values as
`extrinsics_from_yaml` rather than doing a TF lookup, because the vision hosts
have no TF buffer — documented in `CameraStreams.get_camera`, including that this
makes it the *declared* pose and that a USD-vs-yaml divergence is undetected.

---

## Files touched

| File | New? | What changed |
| --- | --- | --- |
| `ros2_ws/src/robot_backend/robot_backend/events/schema.py` | | `LookPayload`; `"look"` added to `EventEnvelope.event_type`, `EVENT_TYPE_PAYLOAD_CLASSES`, `_WATCHDOG_EVENT_TYPES`, and `make_event`'s `Literal` + payload union |
| `ros2_ws/src/robot_backend/robot_backend/watchdog/recorder.py` | | `"look"` added to `_SELF_EMITTED_EVENT_TYPES` |
| `ros2_ws/src/robot_backend/robot_backend/watchdog/image_buffer.py` | | `CameraStreams` + module-level helpers; `ImageBufferManager` gained an optional `streams=` forward |
| `ros2_ws/src/robot_backend/robot_backend/vision/__init__.py` | **new** | package docstring + re-exports |
| `ros2_ws/src/robot_backend/robot_backend/vision/look.py` | **new** | the shared `look()` implementation |
| `ros2_ws/src/robot_backend/robot_backend/mcp_servers/watchdog_mcp_server.py` | | `look()` tool; `camera_streams` on the context; `_IMAGE_RESULT_ANNOTATIONS`; `capture_frames` docstring rewritten |
| `ros2_ws/src/robot_backend/robot_backend/mcp_servers/scene_mcp_server.py` | | `look()` tool; separate vision node/executor/thread; `_LARGE_RESULT_ANNOTATIONS` |
| `RUN.md` | | new `## Vision` section |
| `docs/dev/STAGE1_REPORT.md` | **new** | this file |

`setup.py` needed **no** change: it uses `find_packages()`, so
`robot_backend.vision` is picked up. `robot_backend` is a copy-install package,
so a `colcon build` is still required before the servers see any of this.

---

## New functions and signatures

### `robot_backend/watchdog/image_buffer.py`

Module constants:

```python
_DEFAULT_DEPTH_IMAGE_TOPIC   = "/camera/depth/image_raw"
_DEFAULT_CAMERA_INFO_TOPICS  = ("/camera/depth/camera_info", "/camera_info")
_DEFAULT_POINTCLOUD_TOPIC    = "/camera/depth/points"
DEFAULT_STALE_SEC            = 1.0
CAMERA_EXTRINSICS_FROM_YAML: dict[str, Any]
```

```python
@dataclass
class _Latest:
    msg: Any; stamp_ns: int; wall_ns: int

def _header_stamp_ns(msg: Any) -> int

class CameraStreams:
    def __init__(self, logger: Any | None = None,
                 stale_sec: float = DEFAULT_STALE_SEC) -> None
    def offer_rgb(self, frame: ImageFrame) -> None
    def attach(self, node, callback_group, *, subscribe_rgb: bool = False,
               rgb_candidates: tuple[str, ...] | list[str] | None = None
               ) -> dict[str, Any]
    @staticmethod
    def _resolve_info_topic(types_by_topic: dict[str, Any]) -> str
    def ages(self) -> dict[str, float | None]
    def get_rgb(self) -> tuple[Any, int] | None                  # (HxWx3 uint8, stamp_ns)
    def get_depth(self) -> tuple[Any, int, str] | None           # (HxW f32 m, stamp_ns, source)
    def describe_depth_availability(self) -> str
    def get_camera(self) -> dict[str, Any] | None
    def stats(self) -> dict[str, Any]

def rgb_to_array(data: bytes, width: int, height: int, encoding: str) -> Any | None
def depth_image_to_metres(msg: Any) -> Any | None
def camera_info_to_intrinsics(msg: Any) -> dict[str, Any] | None
def cloud_xyz(msg: Any) -> Any | None                            # Nx3 float32
def project_cloud_to_depth(msg: Any, intrinsics: dict[str, Any]) -> Any | None
```

Changed signature (backward-compatible, new last parameter defaulting to `None`):

```python
ImageBufferManager.__init__(self, buffer, hz=5.0, logger=None, streams=None)
```

`get_camera()` returns
`{frame_id, width, height, K, D, distortion_model, extrinsics_from_yaml:{parent, child, xyz, quat_xyzw, source}, stamp_ns}`.

`get_depth()`'s `source` is `"depth_image"` when `image_raw` is fresher than
`stale_sec`, else `"pointcloud_projection"`. Both encodings `32fc1` (metres) and
`16uc1` (millimetres, `0` → NaN) are read; anything else is refused.

### `robot_backend/vision/look.py`

```python
VALID_VIEWS = ("rgb", "depth")
DEPTH_CLIP_PERCENTILES = (2.0, 98.0)

@dataclass
class LookDeps:
    streams: Any
    session_dir: Path
    role: str
    file_prefix: str
    publish_event: Callable[[Any], None] | None = None
    current_step_id: Callable[[], str | None] | None = None
    logger: Any | None = None
    @property
    def stale_sec(self) -> float      # read from streams, never duplicated

@dataclass
class LookResult:
    look_id, role, reason, views_requested, wall_ns,
    views_ok, views_failed, paths, stamps_ns, ages_ms,
    images: dict[str, tuple[bytes, int, int]],
    depth_source, depth_clip_m, image_size, camera_frame_id,
    camera, step_id, notes
    def text_payload(self) -> dict[str, Any]

def downscale(array: Any, max_width: int) -> Any
def depth_to_grayscale_png(depth: Any) -> tuple[bytes, list[float]] | None
def perform_look(deps: LookDeps, reason: str, views: Any = None,
                 max_width: int = 640) -> LookResult
def build_look_content(result: LookResult) -> list[Any]
def build_look_event(result: LookResult) -> Any
def look(deps: LookDeps | Callable[[], LookDeps], reason: str,
         views: Any = None, max_width: int = 640,
         role: str = "unknown") -> list[Any]
```

Private: `_next_seq()`, `_save()`, `_save_npy()`, `_normalise_views()`,
`_tool_failure()`.

`deps` accepts a **callable** so a host whose setup can fail (`_ensure_watchdog`,
`_ensure_vision`) does that work inside `look()`'s never-raise guard instead of
before it. Both servers use the callable form.

### Tool signatures (identical on both servers)

```python
@mcp.tool(structured_output=False, annotations=<large-result>)
def look(reason: str, views: list[str] | None = None, max_width: int = 640)
```

No return annotation, deliberately — see "FastMCP" below.

### `robot_backend/mcp_servers/scene_mcp_server.py`

```python
_VISION_NODE, _VISION_EXECUTOR, _VISION_THREAD, _CAMERA_STREAMS   # module globals
def _vision_session_dir() -> Path
def _ensure_vision()            # -> CameraStreams | None, idempotent
def _shutdown_vision() -> None  # called first inside _shutdown_backend
```

### `robot_backend/events/schema.py`

`LookPayload` fields: `look_id`, `role`, `reason`, `views_requested`,
`views_ok`, `views_failed`, `paths`, `stamps_ns`, `ages_ms`, `wall_ns`,
`depth_source`, `depth_clip_m`, `image_size`, `camera_frame_id`, `step_id`.
Frozen, every field after the first four optional with a default, following
`FramesCapturedPayload`.

---

## Decisions worth knowing

**FastMCP version and return form.** Installed is **mcp 1.27.1** (note: the
`_venv_comment` in `executor.mcp.json:54` and `watchdog_s2.mcp.json:37` both claim
1.29.1 — stale, not corrected here). Import used:

```python
from mcp.server.fastmcp import Image      # re-exported at server/fastmcp/__init__.py:8
Image(data=png_bytes, format="png")
```

Return form is a plain `list`: `[json_string, Image, Image, ...]`.
`func_metadata._convert_to_content` (lines 507–541) flattens a list and maps each
`Image` to an `ImageContent` block and the leading `str` to a `TextContent`, so
that list is exactly what reaches the model.

`structured_output=False` and the absent return annotation are **required**, not
stylistic. With a return annotation, `convert_result` (lines 98–132) builds an
output model and `model_validate`s the returned value against it; an `Image` is
not pydantic-serialisable and that is where it would fail. `structured_output` is
exposed on `FastMCP.tool` at `server/fastmcp/server.py:454` and survives
`role_context.install_call_logging`, which forwards `*t_args, **t_kwargs`
verbatim (`role_context.py:453-459`).

**Result-size annotation.** A base64 PNG is far past the ~50 KB default per-tool
result cap, and an over-cap result becomes a preview these Read-denied sessions
cannot open — the images would be silently lost. `look()` therefore carries
`anthropic/maxResultSizeChars: 4_000_000`. I added it as a *separate* constant
(`_IMAGE_RESULT_ANNOTATIONS`) rather than raising the existing
`_LARGE_RESULT_ANNOTATIONS` (400 000), so widening the ceiling for images does not
also widen it for `get_evidence` and `get_rendered_task`.

**One RGB subscription, not two.** On the WatchDog, `ImageBufferManager` already
owns an RGB subscription for `capture_frames`; it now forwards each accepted frame
to `CameraStreams.offer_rgb`, so `look()` costs no extra deserialisation.
Forwarding happens *after* the ring append and is wrapped, so it can never cost
`capture_frames` a frame. On the executor there is no ring buffer, so
`CameraStreams.attach(subscribe_rgb=True)` owns RGB there.

**The executor needs its own spinning node.** `scene_mcp_server._NODE` is driven
only by `spin_until_future_complete` inside `_call_service`, for the duration of
one service call — subscriptions on it would receive messages only during
unrelated pose queries. Adding `_NODE` to a background executor is not an option
(double-spin). So vision gets `robot_backend_scene_vision`, its own
`MultiThreadedExecutor(num_threads=2)` and a daemon thread, torn down first in
`_shutdown_backend`.

**`camera_info` has two candidate topics.** The spec names
`/camera/depth/camera_info`, but `sim_moveit_bringup.launch.py:28` records
`/camera_info` (global) in `_IMAGE_BAG_TOPICS`, which is what exists on an
un-updated stage. `_resolve_info_topic` tries the namespaced name first, then the
global one, and falls back to the first for the "missing" report.
`WATCHDOG_CAMERA_INFO_TOPIC` overrides. Subscribing to both was rejected: one
slot filled from two publishers gives no way to say which intrinsics a projection
used.

**Depth visualisation is reported, not just rendered.** Near = bright, linear over
the frame's own [p2, p98] of valid pixels. The clip range in metres goes in the
text part, on the event, and with an explicit note that BLACK is ambiguous
between "no depth measured" and "at/beyond the far clip", and that brightness is
comparable only within one image. The raw float32 array is saved as `.npy`
*before* any visualisation, so later reasoning can be audited against real
distances rather than the grey mapping.

**Downscaling is integer stride, not averaging.** Aspect preserved exactly by
using one factor `ceil(w/max_width)` on both axes. For depth, averaging
neighbouring pixels would produce distances at which no surface exists.

**Never raising is enforced in three places**: setup (callable `deps`), capture,
and content building. A view that is stale/missing/unreadable is a `views_failed`
entry with a reason; a look with no successful view appends an explicit note that
this says nothing about the workspace and must not be read as a clear scene.

**Existing tests checked statically, none appear to break.**
`test_run_ended_terminal_event.py:116` asserts `set(KNOWN_EVENT_TYPES) ==
set(EVENT_TYPE_PAYLOAD_CLASSES)` — both got `"look"`.
`test_scenario_gating.py` uses subset assertions (`<=`) only.
`ImageBufferManager.__init__`'s new parameter is last and defaults to `None`, and
`attach()` is unchanged, so `test_session_and_image_buffer.py` should be
unaffected. **Not run** — see Rule compliance.

---

## Not resolved

1. **Nothing has been executed.** Highest-value first checks, in order: import
   both servers (`_ROBOT_BACKEND_MCP_VENV_READY=1`) to catch import-time errors;
   then `pytest test_session_and_image_buffer.py test_run_ended_terminal_event.py
   test_scenario_gating.py`; then a live `look(reason="smoke")` on a running cell.

2. **ROS message field names are unverified.** `camera_info_to_intrinsics` reads
   `msg.k` with a `msg.K` fallback (rclpy generates lowercase `k`/`d`; the
   fallback covers the other convention). `cloud_xyz` assumes
   `msg.point_step`, `msg.fields[].{name,offset,datatype}`, `msg.data`. These are
   standard but untested here.

3. **Camera resolution and intrinsics are still unread** — they live in the
   binary `assembly_room.usd` (see `VISION_DISCOVERY.md` §1.1, §1.4). Now that
   `/camera/depth/camera_info` publishes, `ros2 topic echo --once` on it is a far
   cheaper way to learn them than decoding the crate, and `look()`'s `camera`
   block reports them at runtime. The **RGB publish rate is ~4.6 Hz**, per
   `sim_moveit_bringup.launch.py:26` — which also means the image ring buffer's
   5 Hz throttle is effectively a no-op and `look()`'s 1 s staleness threshold is
   roughly 4–5 frames' grace.
   Consequences: `max_width=640` is a guess at a sensible default, and I could
   not sanity-check the 512 MB / 2000-frame ring bounds against real frame sizes.
   A depth ring buffer, if Stage 2 wants one, needs its own budget rather than a
   share of that one (float32 is 4 bytes/px).

4. ~~`/camera/depth/image_raw` and `camera_info` do not exist yet~~ —
   **RESOLVED 2026-09-30, see the status update below.** Both are live. What
   remains unverified is that the code reads them correctly, which is item 1.

5. **`LookPayload` has no `notes` field.** Save failures and the
   "no view could be returned" note reach the tool result but not the event. The
   event's `paths` dict being short is the only on-record signal that a save
   failed. Deliberate (keeping the payload to facts), but worth revisiting when
   the scorer starts reading look events in Stage 3/5.

6. **`event_type_origin("look")` reports `"watchdog"` for both roles.** `"look"`
   is the only event type both sides emit, and that function's contract is
   two-valued. `LookPayload.role` is the authoritative discriminator and is
   commented as such in `schema.py`; a third origin value (`"both"`) would be the
   cleaner fix but changes an agent-facing contract, so I left it.

7. **`"look"` is in `_SELF_EMITTED_EVENT_TYPES`**, which correctly stops the
   WatchDog being woken by its own look — but, because the filter is by type and
   cannot see `role`, it also suppresses the *executor's* look events from the
   WatchDog's `wait_for_event`. Whether a WatchDog should be able to wait on the
   executor looking is a real design question I did not want to answer silently
   in Stage 1.

8. **`np.minimum.at` in `project_cloud_to_depth` is the slow path** in numpy for
   scattered reduction. Correct and order-independent, but if a full-resolution
   cloud turns out to be large this is the line to profile — relevant to Stage 2,
   where a 2 Hz array monitor would call it repeatedly.

9. **`describe_depth_availability()` recomputes from current slot state**, so in
   principle it can describe a state that arrived microseconds after the failed
   `get_depth()`. Chosen over remembering the failure (which can go stale behind
   the accessor and is not thread-safe); the window is small and the text is
   explanatory, not load-bearing.

10. **The stale `_venv_comment` figures** in `executor.mcp.json:54` and
    `watchdog_s2.mcp.json:37` (mcp 1.29.1 → actually 1.27.1; trimesh 5.1.0 →
    actually 4.12.2) were left alone, being outside this stage's scope.
