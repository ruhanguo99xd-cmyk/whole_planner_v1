from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tra_planning.msg import ExcavationInfo


def normalize_relative_rotation_deg(current_deg: float, baseline_deg: float) -> float:
    return ((current_deg - baseline_deg + 180.0) % 360.0) - 180.0


def should_finish_due_to_rotation(current_deg: float, baseline_deg: float, limit_deg: float) -> bool:
    return abs(normalize_relative_rotation_deg(current_deg, baseline_deg)) >= limit_deg


def should_finish_due_to_no_result(no_result_count: int, limit_count: int) -> bool:
    return no_result_count >= limit_count


class LegacyDigPlannerOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('legacy_dig_planner_orchestrator')
        self.declare_parameter('status_topic', '/dig/status')
        self.declare_parameter('start_service_name', '/dig/start')
        self.declare_parameter('stop_service_name', '/dig/stop')
        self.declare_parameter('cancel_service_name', '/dig/cancel')
        self.declare_parameter('perception_start_topic', 'digging/perception_start')
        self.declare_parameter('perception_finish_topic', 'digging/perception_finish')
        self.declare_parameter('excavation_start_topic', 'digging/excavation_start')
        self.declare_parameter('cancel_topic', 'digging/cancel')
        self.declare_parameter('excavation_info_topic', 'digging/excavation_end_length')
        self.declare_parameter('rotation_angle_topic', '/lite_slam/swing_angle_deg')
        self.declare_parameter('load_output_file', '')
        self.declare_parameter('return_output_file', '')
        self.declare_parameter('perception_timeout_sec', 20.0)
        self.declare_parameter('planner_timeout_sec', 60.0)
        self.declare_parameter('load_return_timeout_sec', 60.0)
        self.declare_parameter('poll_period_sec', 0.1)
        self.declare_parameter('cycle_pause_sec', 0.5)
        self.declare_parameter('max_consecutive_no_result', 3)
        self.declare_parameter('rotation_limit_deg', 120.0)

        self.status_topic = str(self.get_parameter('status_topic').value)
        self.start_service_name = str(self.get_parameter('start_service_name').value)
        self.stop_service_name = str(self.get_parameter('stop_service_name').value)
        self.cancel_service_name = str(self.get_parameter('cancel_service_name').value)
        self.perception_start_topic = str(self.get_parameter('perception_start_topic').value)
        self.perception_finish_topic = str(self.get_parameter('perception_finish_topic').value)
        self.excavation_start_topic = str(self.get_parameter('excavation_start_topic').value)
        self.cancel_topic = str(self.get_parameter('cancel_topic').value)
        self.excavation_info_topic = str(self.get_parameter('excavation_info_topic').value)
        self.rotation_angle_topic = str(self.get_parameter('rotation_angle_topic').value)
        self.load_output_file = Path(str(self.get_parameter('load_output_file').value)).expanduser()
        self.return_output_file = Path(str(self.get_parameter('return_output_file').value)).expanduser()
        self.perception_timeout_sec = float(self.get_parameter('perception_timeout_sec').value)
        self.planner_timeout_sec = float(self.get_parameter('planner_timeout_sec').value)
        self.load_return_timeout_sec = float(self.get_parameter('load_return_timeout_sec').value)
        self.poll_period_sec = float(self.get_parameter('poll_period_sec').value)
        self.cycle_pause_sec = float(self.get_parameter('cycle_pause_sec').value)
        self.max_consecutive_no_result = int(self.get_parameter('max_consecutive_no_result').value)
        self.rotation_limit_deg = float(self.get_parameter('rotation_limit_deg').value)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.perception_start_pub = self.create_publisher(Bool, self.perception_start_topic, 10)
        self.excavation_start_pub = self.create_publisher(Bool, self.excavation_start_topic, 10)
        self.cancel_pub = self.create_publisher(Bool, self.cancel_topic, 10)
        self.create_subscription(Bool, self.perception_finish_topic, self._on_perception_finish, 10)
        self.create_subscription(ExcavationInfo, self.excavation_info_topic, self._on_excavation_info, 10)
        self.create_subscription(Float32, self.rotation_angle_topic, self._on_rotation_angle, 10)
        self.create_service(Trigger, self.start_service_name, self.start_callback)
        self.create_service(Trigger, self.stop_service_name, self.stop_callback)
        self.create_service(Trigger, self.cancel_service_name, self.cancel_callback)

        self._lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._perception_finished_event = threading.Event()
        self._excavation_info_event = threading.Event()
        self._latest_excavation_info: Optional[ExcavationInfo] = None
        self._worker_thread: Optional[threading.Thread] = None
        self._current_phase = 'idle'
        self._latest_rotation_deg: Optional[float] = None
        self._rotation_baseline_deg: Optional[float] = None
        self._consecutive_no_result = 0
        self._cycle_index = 0
        self._cancel_reason = 'stop'

        self._publish_status('idle', 'ready')
        self.get_logger().info(
            'legacy_dig_planner_orchestrator started: '
            f'load_output={self.load_output_file} return_output={self.return_output_file} '
            f'rotation_topic={self.rotation_angle_topic} rotation_limit_deg={self.rotation_limit_deg}'
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
            self._rotation_baseline_deg = self._latest_rotation_deg
            self._consecutive_no_result = 0
            self._cycle_index = 0
            self._cancel_reason = 'stop'
            self._publish_bool(self.cancel_pub, False)
            self._worker_thread = threading.Thread(target=self._run_cycle_loop, daemon=True)
            self._worker_thread.start()
        response.success = True
        response.message = 'received'
        return response

    def stop_callback(self, request, response):
        del request
        return self._request_shutdown('stop', response)

    def cancel_callback(self, request, response):
        del request
        return self._request_shutdown('cancel', response)

    def _on_perception_finish(self, msg: Bool) -> None:
        if msg.data:
            self._perception_finished_event.set()

    def _on_excavation_info(self, msg: ExcavationInfo) -> None:
        self._latest_excavation_info = msg
        self._excavation_info_event.set()

    def _on_rotation_angle(self, msg: Float32) -> None:
        with self._lock:
            self._latest_rotation_deg = float(msg.data)
            if self._rotation_baseline_deg is None and self._worker_thread is not None and self._worker_thread.is_alive():
                self._rotation_baseline_deg = self._latest_rotation_deg

    def _run_cycle_loop(self) -> None:
        try:
            self._publish_status('starting', 'dig planner session started')
            while not self._cancelled():
                with self._lock:
                    self._cycle_index += 1
                    cycle_index = self._cycle_index
                if self._complete_due_to_rotation_if_needed():
                    return

                cycle_start_ns = time.time_ns()
                self._perception_finished_event.clear()
                self._excavation_info_event.clear()
                self._latest_excavation_info = None

                self._publish_status('rotating_scan', f'cycle={cycle_index} trigger perception')
                self._publish_bool(self.perception_start_pub)

                self._publish_status('waiting_perception', f'cycle={cycle_index} waiting perception_finish')
                perception_state = self._wait_for_event(self._perception_finished_event, self.perception_timeout_sec)
                if perception_state == 'cancelled' or perception_state == 'completed':
                    return
                if perception_state == 'timeout':
                    self._publish_status('failed', f'cycle={cycle_index} timeout waiting for perception_finish')
                    return

                self._publish_status('resetting', f'cycle={cycle_index} waiting excavation_end_length')
                planner_state = self._wait_for_event(self._excavation_info_event, self.planner_timeout_sec)
                if planner_state == 'cancelled' or planner_state == 'completed':
                    return
                if planner_state == 'timeout':
                    if self._handle_no_result(cycle_index, 'planner timeout waiting excavation_end_length'):
                        return
                    continue

                info = self._latest_excavation_info
                if info is None or not info.planning_success:
                    if info is None:
                        detail = 'planner returned empty excavation info'
                    else:
                        detail = 'planner returned no result'
                    if self._handle_no_result(cycle_index, detail):
                        return
                    continue

                self._consecutive_no_result = 0
                excavation_detail = (
                    f'cycle={cycle_index} planning result received '
                    f'excavation_end_length={info.excavation_end_length:.3f}'
                )
                self._publish_status('digging', excavation_detail)
                self._publish_bool(self.excavation_start_pub)

                output_state = self._wait_for_outputs(cycle_start_ns, self.load_return_timeout_sec)
                if output_state == 'cancelled' or output_state == 'completed':
                    return
                if output_state == 'timeout':
                    self._publish_status('failed', f'cycle={cycle_index} timeout waiting for load/return planner outputs')
                    return

                if self._complete_due_to_rotation_if_needed():
                    return
                self._publish_status('cycle_complete', f'cycle={cycle_index} load/return complete, continue digging')
                time.sleep(self.cycle_pause_sec)
        finally:
            if self._cancelled():
                phase = 'canceled' if self._cancel_reason == 'cancel' else 'stopped'
                detail = 'planner cycle canceled' if self._cancel_reason == 'cancel' else 'planner cycle stopped'
                self._publish_status(phase, detail)
            with self._lock:
                self._worker_thread = None
                self._rotation_baseline_deg = None
                self._cancel_reason = 'stop'

    def _handle_no_result(self, cycle_index: int, reason: str) -> bool:
        self._consecutive_no_result += 1
        if should_finish_due_to_no_result(self._consecutive_no_result, self.max_consecutive_no_result):
            self._publish_status(
                'completed',
                f'reason=consecutive_no_result_limit count={self._consecutive_no_result} cycle={cycle_index} last={reason}',
            )
            return True
        self._publish_status(
            'planning_retry',
            f'cycle={cycle_index} no_result_count={self._consecutive_no_result} limit={self.max_consecutive_no_result} last={reason}',
        )
        time.sleep(self.cycle_pause_sec)
        return False

    def _wait_for_event(self, event: threading.Event, timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._cancelled():
                return 'cancelled'
            if self._complete_due_to_rotation_if_needed():
                return 'completed'
            if event.wait(timeout=self.poll_period_sec):
                return 'event'
        return 'timeout'

    def _wait_for_outputs(self, cycle_start_ns: int, timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._cancelled():
                return 'cancelled'
            if self._complete_due_to_rotation_if_needed():
                return 'completed'
            load_ready = self._path_updated_after(self.load_output_file, cycle_start_ns)
            return_ready = self._path_updated_after(self.return_output_file, cycle_start_ns)
            if load_ready and return_ready:
                return 'ready'
            time.sleep(self.poll_period_sec)
        return 'timeout'

    def _complete_due_to_rotation_if_needed(self) -> bool:
        relative_deg = self._relative_rotation_deg()
        if relative_deg is None:
            return False
        if abs(relative_deg) < self.rotation_limit_deg:
            return False
        self._publish_status(
            'completed',
            f'reason=rotation_limit relative_rotation_deg={relative_deg:.2f} limit_deg={self.rotation_limit_deg:.2f}',
        )
        return True

    def _relative_rotation_deg(self) -> Optional[float]:
        with self._lock:
            latest = self._latest_rotation_deg
            baseline = self._rotation_baseline_deg
            if latest is None:
                return None
            if baseline is None:
                self._rotation_baseline_deg = latest
                baseline = latest
        return normalize_relative_rotation_deg(latest, baseline)

    def _publish_bool(self, publisher, value: bool = True) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _publish_status(self, phase: str, detail: str) -> None:
        with self._lock:
            self._current_phase = phase
        msg = String()
        msg.data = f'{phase}|{detail}'
        self.status_pub.publish(msg)
        self.get_logger().info(f'Dig planner status -> {phase}: {detail}')

    def _cancelled(self) -> bool:
        return self._cancel_event.is_set()

    def _request_shutdown(self, mode: str, response):
        self._cancel_event.set()
        with self._lock:
            self._cancel_reason = mode
            running = self._is_running_locked()
        if running:
            self._publish_bool(self.cancel_pub, True)
            phase = 'cancel_requested' if mode == 'cancel' else 'stopping'
            detail = 'cancel requested' if mode == 'cancel' else 'stop requested'
            self._publish_status(phase, detail)
        else:
            self._publish_status('idle', 'planner idle')
        response.success = True
        response.message = 'received'
        return response

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
