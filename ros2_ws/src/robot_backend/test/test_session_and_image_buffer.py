"""Unit tests for the ROS-free session/manifest helper and image ring buffer.

Pure Python -- no cell, no rclpy. Run with:
    pytest test/test_session_and_image_buffer.py -v
"""

from __future__ import annotations

import json

import pytest

from robot_backend.watchdog import session as sess
from robot_backend.watchdog.image_buffer import (
    ImageBufferManager,
    ImageFrame,
    ImageRingBuffer,
)


# --------------------------------------------------------------------------
# session.py -- resolution order and pointer file
# --------------------------------------------------------------------------

@pytest.fixture
def isolated_session(monkeypatch, tmp_path):
    """Isolate every session knob from the real machine: no env, a tmp pointer
    file that does not exist yet, and a tmp session root."""
    monkeypatch.delenv("WATCHDOG_SESSION_DIR", raising=False)
    monkeypatch.setenv("WATCHDOG_SESSION_ROOT", str(tmp_path / "root"))
    monkeypatch.setattr(sess, "_POINTER_FILE", tmp_path / "current_session")
    return tmp_path


def test_resolve_rule_env_wins(monkeypatch, isolated_session, tmp_path):
    monkeypatch.setenv("WATCHDOG_SESSION_DIR", str(tmp_path / "explicit"))
    # even with a pointer present, env wins
    (tmp_path / "ptr_target").mkdir()
    sess.write_pointer(tmp_path / "ptr_target")
    path, rule = sess.resolve_session_dir()
    assert rule == "env"
    assert path == tmp_path / "explicit"


def test_resolve_rule_pointer_when_no_env(isolated_session, tmp_path):
    target = tmp_path / "ptr_target"
    target.mkdir()
    sess.write_pointer(target)
    path, rule = sess.resolve_session_dir()
    assert rule == "pointer"
    assert path == target


def test_resolve_pointer_ignored_when_dir_missing(isolated_session, tmp_path):
    # pointer names a directory that does not exist -> fall through to fresh
    sess.write_pointer(tmp_path / "gone")
    path, rule = sess.resolve_session_dir()
    assert rule == "fresh"
    assert str(path).startswith(str(tmp_path / "root"))


def test_resolve_rule_fresh_under_root(isolated_session, tmp_path):
    path, rule = sess.resolve_session_dir()
    assert rule == "fresh"
    assert path.parent == tmp_path / "root"


def test_write_pointer_is_atomic_and_readable(isolated_session, tmp_path):
    target = tmp_path / "sess1"
    target.mkdir()
    sess.write_pointer(target)
    assert sess.read_pointer() == target
    # no stray temp file left behind
    assert not list((tmp_path).glob(".current_session.*.tmp"))


def test_ensure_session_dir_env(monkeypatch, isolated_session, tmp_path):
    target = tmp_path / "shared"
    monkeypatch.setenv("WATCHDOG_SESSION_DIR", str(target))
    d1 = sess.ensure_session_dir()
    d2 = sess.ensure_session_dir()  # second call must not raise
    assert d1 == d2 == target
    assert target.is_dir()


def test_event_log_default_routes_into_active_session(monkeypatch, isolated_session, tmp_path):
    monkeypatch.setenv("WATCHDOG_SESSION_DIR", str(tmp_path / "s"))
    assert sess.event_log_default("scene") == str(tmp_path / "s" / "events_scene.jsonl")
    assert sess.monitor_log_default() == str(tmp_path / "s" / "events_watchdog.jsonl")
    assert sess.orchestrator_log_dir_default() == str(tmp_path / "s" / "orchestrator")


def test_event_log_default_follows_pointer(isolated_session, tmp_path):
    target = tmp_path / "ptr"
    target.mkdir()
    sess.write_pointer(target)
    assert sess.event_log_default("motion") == str(target / "events_motion.jsonl")


def test_event_log_default_legacy_when_no_env_no_pointer(isolated_session):
    # active_session_dir() is None here -> legacy ~/.watchdog location
    got = sess.event_log_default("motion")
    assert got.endswith("/.watchdog/events_motion.jsonl")


def test_session_root_prefers_env_override(monkeypatch, tmp_path):
    monkeypatch.setenv("WATCHDOG_SESSION_ROOT", str(tmp_path / "custom_root"))
    assert sess.session_root() == tmp_path / "custom_root"


def test_session_root_default_under_agents_transcripts(monkeypatch):
    monkeypatch.delenv("WATCHDOG_SESSION_ROOT", raising=False)
    root = sess.session_root()
    # In this repo checkout the root is located; assert the expected tail.
    if sess.repo_root() is not None:
        assert root.parts[-3:] == ("agents", "transcripts", "sessions")
    else:  # install-only fallback
        assert str(root).endswith("/.watchdog/sessions")


def test_per_server_filenames_do_not_collide(monkeypatch, isolated_session, tmp_path):
    monkeypatch.setenv("WATCHDOG_SESSION_DIR", str(tmp_path / "s"))
    paths = {sess.event_log_default(n) for n in ("scene", "motion", "gripper")}
    assert len(paths) == 3  # each server gets a distinct file in one session


def test_write_manifest_atomic_and_embeds_registry(tmp_path):
    registry = {"objects": {"green_cube": {"enabled": True}}}
    reg_file = tmp_path / "model_registry.yaml"
    reg_file.write_text("objects: {}\n", encoding="utf-8")
    digest = sess.sha256_file(reg_file)

    manifest_path = sess.write_manifest(
        tmp_path,
        registry=registry,
        registry_path=reg_file,
        registry_sha256=digest,
        started_wall_ns=1784850730734067968,
        recorded_topics=["/joint_states", "/tf"],
        images_recorded=False,
        image_throttle_hz=5.0,
        git_commit_value="deadbeef",
    )
    assert manifest_path == tmp_path / "manifest.json"
    data = json.loads(manifest_path.read_text())
    assert data["registry"] == registry
    assert data["registry_sha256"] == digest
    assert data["started_wall_ns"] == 1784850730734067968
    assert data["clock"]["correlation_field"] == "recv_timestamp"
    assert data["clock"]["cell_use_sim_time"] is True
    assert data["clock"]["bag_recorder_use_sim_time"] is False
    assert "header.stamp" in data["clock"]["header_stamp_warning"]
    assert data["images_recorded"] is False
    assert data["git_commit"] == "deadbeef"
    # No stray temp file left behind.
    assert not list(tmp_path.glob(".manifest.json.*.tmp"))


def test_manifest_install_matches_source_true(tmp_path):
    sess.write_manifest(
        tmp_path, registry={"a": 1}, registry_path="/install/reg.yaml",
        registry_sha256="abc", started_wall_ns=1, recorded_topics=[],
        images_recorded=False, image_throttle_hz=5.0,
        registry_source_path="/src/reg.yaml", registry_source_sha256="abc",
    )
    data = json.loads((tmp_path / "manifest.json").read_text())
    assert data["registry_source_path"] == "/src/reg.yaml"
    assert data["registry_source_sha256"] == "abc"
    assert data["registry_install_matches_source"] is True


def test_manifest_install_matches_source_false_on_mismatch(tmp_path):
    sess.write_manifest(
        tmp_path, registry={"a": 1}, registry_path="/install/reg.yaml",
        registry_sha256="install_hash", started_wall_ns=1, recorded_topics=[],
        images_recorded=False, image_throttle_hz=5.0,
        registry_source_path="/src/reg.yaml", registry_source_sha256="source_hash",
    )
    data = json.loads((tmp_path / "manifest.json").read_text())
    assert data["registry_install_matches_source"] is False


def test_manifest_omits_comparison_when_no_source(tmp_path):
    # install-only deployment: source path/hash absent -> comparison is null
    sess.write_manifest(
        tmp_path, registry={"a": 1}, registry_path="/install/reg.yaml",
        registry_sha256="abc", started_wall_ns=1, recorded_topics=[],
        images_recorded=False, image_throttle_hz=5.0,
    )
    data = json.loads((tmp_path / "manifest.json").read_text())
    assert data["registry_source_path"] is None
    assert data["registry_source_sha256"] is None
    assert data["registry_install_matches_source"] is None


def test_write_manifest_first_writer_wins(tmp_path):
    first = sess.write_manifest(
        tmp_path, registry={"a": 1}, registry_path=None, registry_sha256="h1",
        started_wall_ns=1, recorded_topics=[], images_recorded=False,
        image_throttle_hz=5.0,
    )
    # A second write with different content must leave the first untouched.
    sess.write_manifest(
        tmp_path, registry={"b": 2}, registry_path=None, registry_sha256="h2",
        started_wall_ns=2, recorded_topics=[], images_recorded=True,
        image_throttle_hz=9.0,
    )
    data = json.loads(first.read_text())
    assert data["registry"] == {"a": 1}
    assert data["registry_sha256"] == "h1"


# --------------------------------------------------------------------------
# image_buffer.py
# --------------------------------------------------------------------------

def _frame(wall_ns: int, nbytes: int, topic="/camera/depth/rgb", encoding="rgb8"):
    return ImageFrame(
        wall_ns=wall_ns, topic=topic, encoding=encoding, data=b"\x00" * nbytes
    )


def test_ring_buffer_evicts_by_frame_count():
    buf = ImageRingBuffer(max_frames=3, max_bytes=10 ** 9)
    for i in range(5):
        buf.append(_frame(wall_ns=i, nbytes=10))
    assert len(buf) == 3
    rng = buf.buffered_range()
    assert rng == [2, 4]  # oldest two evicted


def test_ring_buffer_evicts_by_byte_budget():
    buf = ImageRingBuffer(max_frames=1000, max_bytes=100)
    for i in range(5):
        buf.append(_frame(wall_ns=i, nbytes=40))  # 40 bytes each
    stats = buf.stats()
    assert stats["total_bytes"] <= 100
    # 100 // 40 -> at most 2 full frames retained
    assert len(buf) <= 2


def test_ring_buffer_retains_single_oversized_frame():
    buf = ImageRingBuffer(max_frames=1000, max_bytes=50)
    buf.append(_frame(wall_ns=1, nbytes=500))  # bigger than the whole budget
    assert len(buf) == 1  # kept rather than dropped to empty


def test_ring_buffer_select_window_inclusive():
    buf = ImageRingBuffer(max_frames=100, max_bytes=10 ** 9)
    for i in range(10):
        buf.append(_frame(wall_ns=i * 100, nbytes=10))
    got = buf.select_window(200, 500)
    assert [f.wall_ns for f in got] == [200, 300, 400, 500]


def test_ring_buffer_window_out_of_range_returns_empty():
    buf = ImageRingBuffer(max_frames=100, max_bytes=10 ** 9)
    for i in range(5):
        buf.append(_frame(wall_ns=1000 + i, nbytes=10))
    assert buf.select_window(0, 100) == []
    assert buf.buffered_range() == [1000, 1004]  # still reports real range


def test_ring_buffer_rejects_nonpositive_bounds():
    with pytest.raises(ValueError):
        ImageRingBuffer(max_frames=0, max_bytes=100)
    with pytest.raises(ValueError):
        ImageRingBuffer(max_frames=10, max_bytes=0)


# --------------------------------------------------------------------------
# ImageBufferManager throttle + topic selection (ROS-free parts)
# --------------------------------------------------------------------------

def test_manager_throttle_accepts_at_target_rate():
    buf = ImageRingBuffer(max_frames=100, max_bytes=10 ** 9)
    mgr = ImageBufferManager(buf, hz=5.0)  # min period 200 ms
    t = "/camera/depth/rgb"
    assert mgr.should_accept(t, 0) is True            # first always accepted
    assert mgr.should_accept(t, 100_000_000) is False  # +100ms, too soon
    assert mgr.should_accept(t, 200_000_000) is True   # +200ms, accepted
    assert mgr.should_accept(t, 250_000_000) is False


def test_manager_throttle_is_per_topic():
    mgr = ImageBufferManager(ImageRingBuffer(10, 10 ** 9), hz=5.0)
    assert mgr.should_accept("/a", 0) is True
    assert mgr.should_accept("/b", 0) is True  # independent topic, not throttled


def test_manager_prefers_compressed_and_dedups_stream():
    types = {
        "/camera/depth/rgb": ["sensor_msgs/msg/Image"],
        "/camera/depth/rgb/compressed": ["sensor_msgs/msg/CompressedImage"],
    }
    candidates = ("/camera/depth/rgb/compressed", "/camera/depth/rgb")
    chosen = ImageBufferManager.choose_topics(candidates, types)
    assert chosen == ["/camera/depth/rgb/compressed"]  # compressed wins, once


def test_manager_falls_back_to_raw_when_no_compressed():
    types = {"/camera/depth/rgb": ["sensor_msgs/msg/Image"]}
    chosen = ImageBufferManager.choose_topics(
        ("/camera/depth/rgb/compressed", "/camera/depth/rgb"), types
    )
    assert chosen == ["/camera/depth/rgb"]


def test_manager_chooses_nothing_when_topics_absent():
    chosen = ImageBufferManager.choose_topics(
        ("/camera/depth/rgb/compressed", "/camera/depth/rgb"), {"/other": ["x"]}
    )
    assert chosen == []
