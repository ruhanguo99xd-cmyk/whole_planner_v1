from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tra_planning.msg import ExcavationInfo


class LegacyDigPlannerOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('legacy_dig_planner_orchestrator')
        self.declare_parameter('status_topic', '/dig/status')
        self.declare_parameter('start_service_name', '/dig/start')
        self.declare_parameter('stop_service_name', '/dig/stop')
        self.declare_parameter('perception_start_topic', 'digging/perception_start')
        self.declare_parameter('perception_finish_topic', 'digging/perception_finish')
        self.declare_parameter('excavation_start_topic', 'digging/excavation_start')
        self.declare_parameter('excavation_info_topic', 'digging/excavation_end_length')
        self.declare_parameter('load_output_file', '')
        self.declare_parameter('return_output_file', '')
        self.declare_parameter('perception_timeout_sec', 20.0)
        self.declare_parameter('planner_timeout_sec', 60.0)
        self.declare_parameter('load_return_timeout_sec', 60.0)
        self.declare_parameter('poll_period_sec', 0.1)

        self.status_topic = str(self.get_parameter('status_topic').value)
        self.start_service_name = str(self.get_parameter('start_service_name').value)
        self.stop_service_name = str(self.get_parameter('stop_service_name').value)
        self.perception_start_topic = str(self.get_parameter('perception_start_topic').value)
        self.perception_finish_topic = str(self.get_parameter('perception_finish_topic').value)
        self.excavation_start_topic = str(self.get_parameter('excavation_start_topic').value)
        self.excavation_info_topic = str(self.get_parameter('excavation_info_topic').value)
        self.load_output_file = Path(str(self.get_parameter('load_output_file').value)).expanduser()
        self.return_output_file = Path(str(self.get_parameter('return_output_file').value)).expanduser()
        self.perception_timeout_sec = float(self.get_parameter('perception_timeout_sec').value)
        self.planner_timeout_sec = float(self.get_parameter('planner_timeout_sec').value)
        self.load_return_timeout_sec = float(self.get_parameter('load_return_timeout_sec').value)
        self.poll_period_sec = float(self.get_parameter('poll_period_sec').value)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.perception_start_pub = self.create_publisher(Bool, self.perception_start_topic, 10)
        self.excavation_start_pub = self.create_publisher(Bool, self.excavation_start_topic, 10)
        self.create_subscription(Bool, self.perception_finish_topic, self._on_perception_finish, 10)
        self.create_subscription(ExcavationInfo, self.excavation_info_topic, self._on_excavation_info, 10)
        self.create_service(Trigger, self.start_service_name, self.start_callback)
        self.create_service(Trigger, self.stop_service_name, self.stop_callback)

        self._lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._perception_finished_event = threading.Event()
        self._excavation_info_event = threading.Event()
        self._latest_excavation_info: Optional[ExcavationInfo] = None
        self._worker_thread: Optional[threading.Thread] = None
        self._current_phase = 'idle'

        self._publish_status('idle', 'ready')
        self.get_logger().info(
            'legacy_dig_planner_orchestrator started: '
            f'load_output={self.load_output_file} return_output={self.return_output_file}'
        )

    def start_callback(self, request, response):
        del request
        with self._lock:
            if self._is_running_locked():
                response.success = False
                response.message = 'dig planner busy'
                return response
            self._cancel_event = threading.Event()
            self._perception_finished_event = threading.Event()
            self._excavation_info_event = threading.Event()
            self._latest_excavation_info = None
            self._worker_thread = threading.Thread(target=self._run_cycle, daemon=True)
            self._worker_thread.start()
        response.success = True
        response.message = 'received'
        return response

    def stop_callback(self, request, response):
        del request
        self._cancel_event.set()
        with self._lock:
            running = self._is_running_locked()
        if running:
            self._publish_status('stopping', 'stop requested')
        else:
            self._publish_status('idle', 'planner idle')
        response.success = True
        response.message = 'received'
        return response

    def _on_perception_finish(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._perception_finished_event.set()

    def _on_excavation_info(self, msg: ExcavationInfo) -> None:
        self._latest_excavation_info = msg
        self._excavation_info_event.set()

    def _run_cycle(self) -> None:
        cycle_start_ns = time.time_ns()
        try:
            self._publish_status('starting', 'dig planner cycle started')
            if self._cancelled():
                return

            self._publish_status('rotating_scan', 'triggering trajectory planner perception stage')
            self._publish_bool(self.perception_start_pub)

            self._publish_status('waiting_perception', 'waiting for trajectory planner perception_finish')
            if not self._wait_for_event(self._perception_finished_event, self.perception_timeout_sec):
                if self._cancelled():
                    return
                self._publish_status('failed', 'timeout waiting for perception_finish')
                return

            self._publish_status('resetting', 'waiting for excavation_end_length from trajectory planner')
            if not self._wait_for_event(self._excavation_info_event, self.planner_timeout_sec):
                if self._cancelled():
                    return
                self._publish_status('failed', 'timeout waiting for excavation_end_length')
                return

            info = self._latest_excavation_info
            detail = 'received excavation_end_length'
            if info is not None and not info.planning_success:
                detail = 'received fallback excavation_end_length after planning failure'
            self._publish_status('digging', detail)
            self._publish_bool(self.excavation_start_pub)

            if not self._wait_for_outputs(cycle_start_ns, self.load_return_timeout_sec):
                if self._cancelled():
                    return
                self._publish_status('failed', 'timeout waiting for load/return planner outputs')
                return

            self._publish_status('completed', 'trajectory_planner, load and return completed')
        finally:
            if self._cancelled():
                self._publish_status('stopped', 'planner cycle stopped')
            with self._lock:
                self._worker_thread = None

    def _wait_for_event(self, event: threading.Event, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._cancelled():
                return False
            if event.wait(timeout=self.poll_period_sec):
                return True
        return False

    def _wait_for_outputs(self, cycle_start_ns: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._cancelled():
                return False
            load_ready = self._path_updated_after(self.load_output_file, cycle_start_ns)
            return_ready = self._path_updated_after(self.return_output_file, cycle_start_ns)
            if load_ready and return_ready:
                return True
            time.sleep(self.poll_period_sec)
        return False

    def _publish_bool(self, publisher) -> None:
        msg = Bool()
        msg.data = True
        publisher.publish(msg)

    def _publish_status(self, phase: str, detail: str) -> None:
        with self._lock:
            self._current_phase = phase
        msg = String()
        msg.data = phase
        self.status_pub.publish(msg)
        self.get_logger().info(f'Dig planner status -> {phase}: {detail}')

    def _cancelled(self) -> bool:
        return self._cancel_event.is_set()

    def _is_running_locked(self) -> bool:
        return self._worker_thread is not None and self._worker_thread.is_alive()

    @staticmethod
    def _path_updated_after(path: Path, started_at_ns: int) -> bool:
        try:
            return path.exists() and path.stat().st_mtime_ns >= started_at_ns
        except FileNotFoundError:
            return False


def main() -> None:
    rclpy.init()
    node = LegacyDigPlannerOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
