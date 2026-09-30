"""End-to-end camera receive path, in-process, on an isolated ROS domain.

The failure this pins: attach() used to gate every camera subscription on a
get_topic_names_and_types() snapshot taken milliseconds after node creation,
which races DDS discovery -- a topic not yet discovered was never subscribed,
and look() stayed blind for the whole run while the camera published happily
(observed live 2026-09-30). These tests build the REAL pipelines -- the
watchdog's via watchdog_mcp_server._build_camera_pipeline, the executor's via
scene_mcp_server._ensure_vision -- create the publishers only AFTERWARDS
(modelling the cold-graph attach), and require every view to work anyway.

Everything runs on ROS_DOMAIN_ID=77 with LOCALHOST discovery so nothing here
can touch a live cell on the default domain.
"""

from __future__ import annotations

import os
import threading
import time

import pytest

os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")
os.environ.setdefault(
    "EXECUTOR_TASK_PATH",
    str(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                     "agents", "tasks", "fmb_assembly.executor.yaml")),
)

import numpy as np  # noqa: E402

from robot_backend.watchdog.image_buffer import CameraStreams, ImageBufferManager  # noqa: E402
from robot_backend.vision.look import LookDeps, perform_look  # noqa: E402

rclpy = pytest.importorskip("rclpy")

RGB_TOPIC = "/camera/depth/rgb"
DEPTH_TOPIC = "/camera/depth/image_raw"
INFO_TOPIC = "/camera/depth/camera_info"
FX = 554.26


@pytest.fixture(scope="module")
def ros():
    """One rclpy context for the module, torn down at the end."""
    if not rclpy.ok():
        rclpy.init(args=None)
    yield rclpy
    # Deliberately no rclpy.shutdown(): scene's vision thread and any executor
    # threads are daemonic, and other test modules must not inherit a dead
    # global context if they ever grow ROS tests.


class _FakeCamera:
    """Publishes synthetic rgb/depth/camera_info on the real topic names."""

    def __init__(self, node_name="fake_isaac"):
        from sensor_msgs.msg import CameraInfo, Image

        self.node = rclpy.create_node(node_name)
        self._Image, self._CameraInfo = Image, CameraInfo
        self.rgb_pub = self.node.create_publisher(Image, RGB_TOPIC, 1)
        self.depth_pub = self.node.create_publisher(Image, DEPTH_TOPIC, 1)
        self.info_pub = self.node.create_publisher(CameraInfo, INFO_TOPIC, 1)

    def publish_once(self):
        rgb = self._Image()
        rgb.height, rgb.width, rgb.encoding, rgb.step = 360, 640, "rgb8", 640 * 3
        rgb.data = bytes(360 * 640 * 3)
        depth = self._Image()
        depth.height, depth.width, depth.encoding, depth.step = 360, 640, "32FC1", 640 * 4
        depth.data = np.full((360, 640), 1.5, np.float32).tobytes()
        info = self._CameraInfo()
        info.height, info.width = 360, 640
        info.k = [FX, 0.0, 320.0, 0.0, FX, 180.0, 0.0, 0.0, 1.0]
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)
        self.info_pub.publish(info)

    def publish_for(self, seconds=1.0, hz=10.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_once()
            time.sleep(1.0 / hz)

    def destroy(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass


def _wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def _assert_look_sees(streams, session_dir, role):
    """The shared 3a/3b assertion block: both views ok off live data."""
    deps = LookDeps(
        streams=streams, session_dir=session_dir, role=role,
        file_prefix="test_look_", publish_event=None,
        current_step_id=None, logger=None,
    )
    result = perform_look(deps, reason="camera receive test", views=["rgb", "depth"])
    assert result.views_ok == ["rgb", "depth"], (
        f"views_failed={result.views_failed} streams={streams.stats()}"
    )
    assert result.depth_source == "depth_image"
    assert result.camera is not None
    flat_k = np.asarray(result.camera["K"], dtype=float).flatten().tolist()
    assert FX in flat_k
    stats = streams.stats()["streams"]
    for name in ("rgb", "depth", "camera_info"):
        assert stats[name]["received"] > 0, f"{name}: {stats[name]}"
        assert stats[name]["errors"] == 0, f"{name}: {stats[name]}"
    return result


def test_watchdog_camera_path_end_to_end(ros, tmp_path):
    """3a: the watchdog pipeline, built by the server's own helper, sees the
    camera even though the publishers appeared only after attach."""
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    import robot_backend.mcp_servers.watchdog_mcp_server as ws

    node = rclpy.create_node("test_watchdog_cam")
    # The real construction path: cold graph, attach FIRST.
    buffer, manager, streams = ws._build_camera_pipeline(node, ReentrantCallbackGroup())
    assert RGB_TOPIC in manager.subscribed_topics
    assert DEPTH_TOPIC in streams.subscribed_topics
    assert INFO_TOPIC in streams.subscribed_topics

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    cam = _FakeCamera("fake_isaac_wd")
    try:
        cam.publish_for(seconds=1.2, hz=10.0)
        assert _wait_until(
            lambda: all(
                streams.ages()[k] is not None for k in ("rgb", "depth", "camera_info")
            )
        ), f"slots never filled: {streams.stats()}"
        result = _assert_look_sees(streams, tmp_path, "watchdog")
        # The rgb slot was filled through the manager's forward, not a second
        # subscription of the streams' own.
        assert streams.stats()["streams"]["rgb"]["topic"] == RGB_TOPIC
        assert result.streams is not None  # stats travel on the look itself
    finally:
        cam.destroy()
        executor.shutdown()
        node.destroy_node()


def test_scene_camera_path_end_to_end(ros, tmp_path):
    """3b: the executor's vision path (scene_mcp_server._ensure_vision)."""
    import robot_backend.mcp_servers.scene_mcp_server as sc

    streams = sc._ensure_vision()
    assert streams is not None
    assert RGB_TOPIC in streams.subscribed_topics
    assert DEPTH_TOPIC in streams.subscribed_topics
    assert INFO_TOPIC in streams.subscribed_topics
    cam = _FakeCamera("fake_isaac_scene")
    try:
        cam.publish_for(seconds=1.2, hz=10.0)
        assert _wait_until(
            lambda: all(
                streams.ages()[k] is not None for k in ("rgb", "depth", "camera_info")
            )
        ), f"slots never filled: {streams.stats()}"
        _assert_look_sees(streams, tmp_path, "executor")
    finally:
        cam.destroy()
        sc._shutdown_vision()


# --- 3c: attach never gates on discovery --------------------------------------

class _FakeNode:
    """A graph that reports NO topics, recording what gets subscribed anyway."""

    def __init__(self):
        self.subscriptions = []

    def get_topic_names_and_types(self):
        return []

    def create_subscription(self, msg_cls, topic, handler, depth, callback_group=None):
        self.subscriptions.append((msg_cls.__name__, topic))
        return object()

    def get_logger(self):
        return None


def test_attach_subscribes_unconditionally_on_an_empty_graph():
    node = _FakeNode()
    streams = CameraStreams()
    result = streams.attach(node, None, subscribe_rgb=True)
    topics = [t for _, t in node.subscriptions]
    assert RGB_TOPIC in topics
    assert DEPTH_TOPIC in topics
    # BOTH camera_info candidates, not just the discovered one (none is).
    assert INFO_TOPIC in topics
    assert "/camera_info" in topics
    assert "/camera/depth/points" in topics
    assert topics == result["subscribed"] == streams.subscribed_topics
    # The compressed rgb candidate is still skipped (no codec), by name.
    assert "/camera/depth/rgb/compressed" not in topics


def test_manager_attach_subscribes_unconditionally_on_an_empty_graph():
    from robot_backend.watchdog.image_buffer import ImageRingBuffer

    node = _FakeNode()
    manager = ImageBufferManager(ImageRingBuffer(max_frames=4, max_bytes=1 << 20))
    chosen = manager.attach(node, None)
    assert chosen == [RGB_TOPIC]
    assert [t for _, t in node.subscriptions] == [RGB_TOPIC]


def test_never_received_text_names_topic_counts_and_attach_time():
    node = _FakeNode()
    streams = CameraStreams()
    streams.attach(node, None)
    text = streams.never_received_text("depth")
    assert DEPTH_TOPIC in text
    assert "since attach at" in text
    assert "received=0" in text
    assert "errors=0" in text
