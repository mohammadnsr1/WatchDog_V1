"""Regression test for halt-on-fire from a real subscription callback."""

from __future__ import annotations

import threading
import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.halt import HaltClient
from robot_backend.watchdog.monitors import EffortEventMonitor
from robot_backend.watchdog.recorder import EvidenceRecorder


class _RecordingBus:
    def __init__(self) -> None:
        self.events = []
        self.lock = threading.Lock()

    def publish(self, event) -> None:
        with self.lock:
            self.events.append(event)


def test_halt_response_completes_while_joint_callback_is_blocked() -> None:
    """A subscription-originated fire must not starve its cancel response."""
    context = Context()
    rclpy.init(context=context)
    watchdog_node = Node("watchdog_halt_callback_test", context=context)
    stub_node = Node("watchdog_cancel_stub", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)

    def _cancel(_request, response):
        response.return_code = CancelGoal.Response.ERROR_NONE
        response.goals_canceling = []
        return response

    cancel_service = stub_node.create_service(
        CancelGoal,
        "/move_action/_action/cancel_goal",
        _cancel,
    )
    publisher = stub_node.create_publisher(JointState, "/joint_states", 10)
    bus = _RecordingBus()
    client_group = MutuallyExclusiveCallbackGroup()
    halt_client = HaltClient(
        node=watchdog_node,
        event_bus=bus,
        callback_group=client_group,
    )
    recorder = EvidenceRecorder(node=watchdog_node, registry={})
    engine = MonitorEngine(event_bus=bus, halt_client=halt_client)
    recorder.engine = engine
    engine.arm(
        EffortEventMonitor(
            {
                "joint_name": "test_effort_joint",
                "comparison": "le",
                "threshold": 4.0,
                "sustained_ticks": 1,
            },
            step_id="callback_group_step",
            halt_on_fire=True,
            monitor_id="callback_group_monitor",
        )
    )

    executor.add_node(watchdog_node)
    executor.add_node(stub_node)
    spin_thread.start()
    try:
        deadline = time.monotonic() + 2.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert publisher.get_subscription_count() >= 1

        baseline = JointState()
        baseline.name = ["test_effort_joint"]
        baseline.position = [0.0]
        baseline.effort = [0.0]
        publisher.publish(baseline)
        deadline = time.monotonic() + 1.0
        while not recorder.joint_samples and time.monotonic() < deadline:
            time.sleep(0.01)
        assert recorder.joint_samples

        violation = JointState()
        violation.name = ["test_effort_joint"]
        violation.position = [0.0]
        violation.effort = [4.5]
        publisher.publish(violation)

        def _publish_after_fire() -> None:
            for _ in range(20):
                publisher.publish(baseline)
                time.sleep(0.01)

        post_thread = threading.Thread(target=_publish_after_fire, daemon=True)
        post_thread.start()

        fires = []
        deadline = time.monotonic() + 3.0
        while not fires and time.monotonic() < deadline:
            fires = engine.pop_fires()
            time.sleep(0.01)
        assert fires, "synthetic /joint_states violation did not fire"
        fire = fires[0]
        halt_result = fire["halt_result"]
        assert fire["action_taken"] in {"halt", "halt_no_active_goal"}, fire
        assert not (
            fire["action_taken"] == "halt_failed"
            and "timed out" in halt_result["detail"]
        ), fire
        assert halt_result["elapsed_sec"] < 0.5, halt_result
        post_thread.join(timeout=1.0)
        time.sleep(0.05)
        trigger_ns = fire["trigger_wall_ns"]
        with recorder.lock:
            sample_stamps = [sample[0] for sample in recorder.joint_samples]
        before_ns = max(stamp for stamp in sample_stamps if stamp <= trigger_ns)
        after_ns = min(stamp for stamp in sample_stamps if stamp > trigger_ns)
        callback_stall_sec = (after_ns - before_ns) / 1_000_000_000
        print(
            f"halt_elapsed_sec={halt_result['elapsed_sec']:.6f} "
            f"callback_sample_gap_sec={callback_stall_sec:.6f}"
        )
        assert callback_stall_sec < 0.5
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        watchdog_node.destroy_node()
        stub_node.destroy_service(cancel_service)
        stub_node.destroy_node()
        context.shutdown()
