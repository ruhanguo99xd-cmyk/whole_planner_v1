import threading
import time
from typing import Optional

import rclpy
from integrated_mission_interfaces.action import DigMission
from integrated_mission_interfaces.msg import SubsystemStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from excavation_planner_core.mock_runtime import resolve_mock_behavior

COMMAND_START = 1
COMMAND_STOP = 2
COMMAND_CANCEL = 3

ERROR_BUSY = 1
ERROR_BACKEND_UNAVAILABLE = 2
ERROR_BACKEND_FAILED = 5

_STATUS_PROGRESS = {
    'idle': 0.0,
    'starting': 0.05,
    'waiting_perception': 0.2,
    'rotating_scan': 0.15,
    'planning_retry': 0.3,
    'cycle_complete': 0.85,
    'resetting': 0.4,
    'digging': 0.7,
    'completed': 1.0,
    'stopping': 0.1,
    'stopped': 0.0,
    'cancel_requested': 0.05,
    'canceled': 0.0,
    'failed': 1.0,
}


def _parse_legacy_status(raw_status: str) -> tuple[str, str]:
    status_text = raw_status.strip()
    if '|' not in status_text:
        status = status_text.lower()
        return status, f'legacy dig status={status}'
    phase, detail = status_text.split('|', 1)
    status = phase.strip().lower()
    detail_text = detail.strip() or f'legacy dig status={status}'
    return status, detail_text


class ExcavationActionServer(Node):
    def __init__(self) -> None:
        super().__init__('excavation_planner_core')
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('mock_duration_sec', 5.0)
        self.declare_parameter('legacy_start_service_name', '/dig/start')
        self.declare_parameter('legacy_stop_service_name', '/dig/stop')
        self.declare_parameter('legacy_cancel_service_name', '/dig/cancel')
        self.declare_parameter('legacy_status_topic', '/dig/status')
        self.backend = str(self.get_parameter('backend').value)
        self.mock_duration_sec = float(self.get_parameter('mock_duration_sec').value)
        self.legacy_start_service_name = str(self.get_parameter('legacy_start_service_name').value)
        self.legacy_stop_service_name = str(self.get_parameter('legacy_stop_service_name').value)
        self.legacy_cancel_service_name = str(self.get_parameter('legacy_cancel_service_name').value)
        self.legacy_status_topic = str(self.get_parameter('legacy_status_topic').value)

        self.status_pub = self.create_publisher(SubsystemStatus, '/excavation/status', 10)
        self.create_subscription(String, self.legacy_status_topic, self._on_legacy_status, 10)
        self._legacy_start_client = self.create_client(Trigger, self.legacy_start_service_name, callback_group=self.callback_group)
        self._legacy_stop_client = self.create_client(Trigger, self.legacy_stop_service_name, callback_group=self.callback_group)
        self._legacy_cancel_client = self.create_client(Trigger, self.legacy_cancel_service_name, callback_group=self.callback_group)

        self.current_mission_id = ''
        self.current_phase = 'idle'
        self.current_error_code = 0
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.worker_thread: Optional[threading.Thread] = None
        self.shutdown_mode: Optional[str] = None

        self.action_server = ActionServer(
            self,
            DigMission,
            '/excavation/execute',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._publish_status('idle', active=False, ready=True, error=False, detail='ready', mission_id='')
        self.get_logger().info(f'excavation_planner_core started: backend={self.backend}')

    def goal_callback(self, goal_request) -> GoalResponse:
        if goal_request.command not in (COMMAND_START, COMMAND_STOP, COMMAND_CANCEL):
            return GoalResponse.REJECT
        if goal_request.command == COMMAND_START and goal_request.timeout_sec <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.REJECT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        if goal.command == COMMAND_START:
            result = self._handle_start(goal)
        elif goal.command == COMMAND_STOP:
            result = self._handle_shutdown('stop')
        else:
            result = self._handle_shutdown('cancel')
        if result.accepted:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _handle_start(self, goal) -> DigMission.Result:
        with self.lock:
            if self._is_busy_locked():
                return self._build_result(False, ERROR_BUSY, 'excavation busy')
            self.current_mission_id = goal.mission_id
            self.current_phase = 'starting'
            self.current_error_code = 0
            self.shutdown_mode = None

        if self.backend == 'mock':
            accepted, message, error_code = self._start_mock(goal)
        elif self.backend == 'legacy_dig_command':
            accepted, message, error_code = self._start_legacy(goal.mission_id)
        else:
            accepted, message, error_code = False, f'unsupported backend: {self.backend}', ERROR_BACKEND_UNAVAILABLE

        if accepted:
            return self._build_result(True, 0, 'received')

        with self.lock:
            self.current_mission_id = ''
            self.current_phase = 'idle'
            self.current_error_code = error_code
            self.shutdown_mode = None
        self._publish_status('failed', active=False, ready=False, error=True, detail=message, error_code=error_code)
        return self._build_result(False, error_code, message)

    def _handle_shutdown(self, shutdown_mode: str) -> DigMission.Result:
        if self.backend == 'mock':
            accepted, message, error_code = self._shutdown_mock(shutdown_mode)
        elif self.backend == 'legacy_dig_command':
            accepted, message, error_code = self._shutdown_legacy(shutdown_mode)
        else:
            accepted, message, error_code = False, f'unsupported backend: {self.backend}', ERROR_BACKEND_UNAVAILABLE

        if accepted:
            return self._build_result(True, 0, 'received')
        return self._build_result(False, error_code, message)

    def _start_mock(self, goal) -> tuple[bool, str, int]:
        behavior = resolve_mock_behavior(goal.process_parameters_json, self.mock_duration_sec)
        self.cancel_event = threading.Event()
        self.worker_thread = threading.Thread(
            target=self._run_mock_worker,
            args=(goal.mission_id, behavior.outcome, behavior.duration_sec),
            daemon=True,
        )
        self.worker_thread.start()
        return True, 'received', 0

    def _run_mock_worker(self, mission_id: str, outcome: str, duration_sec: float) -> None:
        start = time.monotonic()
        self._publish_status('starting', active=True, ready=False, error=False, detail='mock dig started', mission_id=mission_id)
        while not self.cancel_event.is_set():
            elapsed = time.monotonic() - start
            if outcome == 'timeout' and elapsed >= duration_sec:
                self._publish_status('digging', active=True, ready=False, error=False, detail='mock dig waiting', progress=0.95, mission_id=mission_id)
                time.sleep(0.2)
                continue
            if elapsed >= duration_sec:
                break
            progress = min(elapsed / max(duration_sec, 0.1), 0.95)
            self._publish_status('digging', active=True, ready=False, error=False, detail='mock dig running', progress=progress, mission_id=mission_id)
            time.sleep(0.2)
        if self.cancel_event.is_set():
            with self.lock:
                shutdown_mode = self.shutdown_mode or 'cancel'
            phase = 'canceled' if shutdown_mode == 'cancel' else 'stopped'
            detail = f'mock dig {phase}'
            self._publish_status(phase, active=False, ready=True, error=False, detail=detail, mission_id=mission_id, clear_mission=True)
            return
        if outcome == 'fail':
            self._publish_status('failed', active=False, ready=False, error=True, detail='mock dig failed', error_code=ERROR_BACKEND_FAILED, mission_id=mission_id)
            return
        self._publish_status('completed', active=False, ready=True, error=False, detail='mock dig completed', progress=1.0, mission_id=mission_id)

    def _start_legacy(self, mission_id: str) -> tuple[bool, str, int]:
        if not self._legacy_start_client.wait_for_service(timeout_sec=5.0):
            return False, 'legacy dig start service unavailable', ERROR_BACKEND_UNAVAILABLE
        future = self._legacy_start_client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        response = future.result()
        if not response.success:
            return False, response.message or 'legacy dig rejected', ERROR_BACKEND_FAILED
        self._publish_status('starting', active=True, ready=False, error=False, detail='legacy dig accepted', mission_id=mission_id)
        return True, 'received', 0

    def _shutdown_mock(self, shutdown_mode: str) -> tuple[bool, str, int]:
        self.cancel_event.set()
        with self.lock:
            self.shutdown_mode = shutdown_mode
            mission_id = self.current_mission_id
            running = self.worker_thread is not None and self.worker_thread.is_alive()
        if not mission_id and not running:
            self._publish_status('idle', active=False, ready=True, error=False, detail='dig idle', mission_id='', clear_mission=True)
            return True, 'received', 0
        phase = 'cancel_requested' if shutdown_mode == 'cancel' else 'stopping'
        detail = 'mock dig cancel requested' if shutdown_mode == 'cancel' else 'mock dig stopping'
        self._publish_status(phase, active=True, ready=False, error=False, detail=detail, mission_id=mission_id)
        return True, 'received', 0

    def _shutdown_legacy(self, shutdown_mode: str) -> tuple[bool, str, int]:
        with self.lock:
            self.shutdown_mode = shutdown_mode
            mission_id = self.current_mission_id
        if not mission_id:
            self._publish_status('idle', active=False, ready=True, error=False, detail='dig idle', mission_id='', clear_mission=True)
            return True, 'received', 0
        phase = 'cancel_requested' if shutdown_mode == 'cancel' else 'stopping'
        detail = 'legacy dig cancel requested' if shutdown_mode == 'cancel' else 'legacy dig stopping'
        self._publish_status(phase, active=True, ready=False, error=False, detail=detail, mission_id=mission_id)
        client = self._legacy_cancel_client if shutdown_mode == 'cancel' else self._legacy_stop_client
        if not client.wait_for_service(timeout_sec=1.0):
            if shutdown_mode == 'cancel' and self._legacy_stop_client.wait_for_service(timeout_sec=1.0):
                client = self._legacy_stop_client
            else:
                return False, 'legacy dig stop service unavailable', ERROR_BACKEND_UNAVAILABLE
        future = client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        response = future.result()
        if not response.success:
            return False, response.message or 'legacy dig stop rejected', ERROR_BACKEND_FAILED
        return True, 'received', 0

    def _on_legacy_status(self, msg: String) -> None:
        status, detail = _parse_legacy_status(msg.data)
        phase_map = {
            'idle': ('idle', False, True, False),
            'starting': ('starting', True, False, False),
            'rotating_scan': ('rotating_scan', True, False, False),
            'waiting_perception': ('waiting_perception', True, False, False),
            'planning_retry': ('planning_retry', True, False, False),
            'resetting': ('resetting', True, False, False),
            'digging': ('digging', True, False, False),
            'cycle_complete': ('cycle_complete', True, False, False),
            'completed': ('completed', False, True, False),
            'stopping': ('stopping', True, False, False),
            'stopped': ('stopped', False, True, False),
            'cancel_requested': ('cancel_requested', True, False, False),
            'canceled': ('canceled', False, True, False),
            'failed': ('failed', False, False, True),
        }
        phase, active, ready, error = phase_map.get(status, ('digging', True, False, False))
        with self.lock:
            shutdown_mode = self.shutdown_mode
        if shutdown_mode == 'cancel' and phase == 'stopped':
            phase = 'canceled'
            detail = f'{detail} (mapped from stop after cancel request)'
        error_code = ERROR_BACKEND_FAILED if error else 0
        clear_mission = phase in {'idle', 'stopped', 'canceled'}
        self._publish_status(
            phase,
            active=active,
            ready=ready,
            error=error,
            detail=detail,
            error_code=error_code,
            progress=_STATUS_PROGRESS.get(phase, 0.5),
            clear_mission=clear_mission,
        )

    def _is_busy_locked(self) -> bool:
        if self.current_mission_id == '':
            return False
        return self.current_phase not in {'idle', 'stopped', 'canceled'}

    def _publish_status(
        self,
        phase: str,
        *,
        active: bool,
        ready: bool,
        error: bool,
        detail: str,
        error_code: int = 0,
        progress: Optional[float] = None,
        mission_id: Optional[str] = None,
        clear_mission: bool = False,
    ) -> None:
        with self.lock:
            mission_value = self.current_mission_id if mission_id is None else mission_id
            self.current_phase = phase
            self.current_error_code = error_code
        msg = SubsystemStatus()
        msg.mission_id = mission_value
        msg.active = active
        msg.ready = ready
        msg.error = error
        msg.error_code = error_code
        msg.phase = phase
        msg.detail = detail
        msg.progress = _STATUS_PROGRESS.get(phase, 0.0) if progress is None else float(progress)
        msg.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)
        if clear_mission:
            with self.lock:
                self.current_mission_id = ''
                self.current_phase = 'idle'
                self.current_error_code = 0
                self.shutdown_mode = None

    @staticmethod
    def _build_result(accepted: bool, error_code: int, message: str) -> DigMission.Result:
        result = DigMission.Result()
        result.accepted = accepted
        result.error_code = error_code
        result.message = message
        return result


def main() -> None:
    rclpy.init()
    node = ExcavationActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_event.set()
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
