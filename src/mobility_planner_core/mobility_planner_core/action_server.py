import importlib
import threading
import time
from typing import Optional

import rclpy
from integrated_mission_interfaces.action import WalkMission
from integrated_mission_interfaces.msg import SubsystemStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mobility_planner_core.mock_runtime import resolve_mock_behavior

COMMAND_START = 1
COMMAND_STOP = 2

ERROR_BUSY = 1
ERROR_BACKEND_UNAVAILABLE = 2
ERROR_BACKEND_FAILED = 5

_STATUS_PROGRESS = {
    'idle': 0.0,
    'starting': 0.05,
    'walking': 0.6,
    'completed': 1.0,
    'stopping': 0.1,
    'stopped': 0.0,
    'failed': 1.0,
}


class MobilityActionServer(Node):
    def __init__(self) -> None:
        super().__init__('mobility_planner_core')
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('mock_duration_sec', 4.0)
        self.declare_parameter('legacy_service_name', '/autonomous_walk/set_goal')
        self.declare_parameter('legacy_stop_service_name', '/autonomous_walk/stop')
        self.declare_parameter('legacy_status_topic', '/autonomous_walk/status')
        self.backend = str(self.get_parameter('backend').value)
        self.mock_duration_sec = float(self.get_parameter('mock_duration_sec').value)
        self.legacy_service_name = str(self.get_parameter('legacy_service_name').value)
        self.legacy_stop_service_name = str(self.get_parameter('legacy_stop_service_name').value)
        self.legacy_status_topic = str(self.get_parameter('legacy_status_topic').value)

        self.status_pub = self.create_publisher(SubsystemStatus, '/mobility/status', 10)
        self.create_subscription(String, self.legacy_status_topic, self._on_legacy_status, 10)
        self._legacy_stop_client = self.create_client(Trigger, self.legacy_stop_service_name, callback_group=self.callback_group)

        self.current_mission_id = ''
        self.current_phase = 'idle'
        self.current_error_code = 0
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.worker_thread: Optional[threading.Thread] = None

        self.action_server = ActionServer(
            self,
            WalkMission,
            '/mobility/execute',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._publish_status('idle', active=False, ready=True, error=False, detail='ready', mission_id='')
        self.get_logger().info(f'mobility_planner_core started: backend={self.backend}')

    def goal_callback(self, goal_request) -> GoalResponse:
        if goal_request.command not in (COMMAND_START, COMMAND_STOP):
            self.get_logger().warning('Reject walk command with invalid command id')
            return GoalResponse.REJECT
        if goal_request.command == COMMAND_START and goal_request.timeout_sec <= 0.0:
            self.get_logger().warning('Reject walk start command with invalid timeout')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.REJECT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        if goal.command == COMMAND_START:
            result = self._handle_start(goal)
        else:
            result = self._handle_stop(goal)
        if result.accepted:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _handle_start(self, goal) -> WalkMission.Result:
        with self.lock:
            if self._is_busy_locked():
                return self._build_result(False, ERROR_BUSY, 'mobility busy')
            self.current_mission_id = goal.mission_id
            self.current_phase = 'starting'
            self.current_error_code = 0

        if self.backend == 'mock':
            accepted, message, error_code = self._start_mock(goal)
        elif self.backend == 'legacy_autonomous_walk_service':
            accepted, message, error_code = self._start_legacy(goal)
        else:
            accepted, message, error_code = False, f'unsupported backend: {self.backend}', ERROR_BACKEND_UNAVAILABLE

        if accepted:
            return self._build_result(True, 0, 'received')

        with self.lock:
            self.current_mission_id = ''
            self.current_phase = 'idle'
            self.current_error_code = error_code
        self._publish_status('failed', active=False, ready=False, error=True, detail=message, error_code=error_code)
        return self._build_result(False, error_code, message)

    def _handle_stop(self, goal) -> WalkMission.Result:
        del goal
        if self.backend == 'mock':
            accepted, message, error_code = self._stop_mock()
        elif self.backend == 'legacy_autonomous_walk_service':
            accepted, message, error_code = self._stop_legacy()
        else:
            accepted, message, error_code = False, f'unsupported backend: {self.backend}', ERROR_BACKEND_UNAVAILABLE

        if accepted:
            return self._build_result(True, 0, 'received')
        return self._build_result(False, error_code, message)

    def _start_mock(self, goal) -> tuple[bool, str, int]:
        behavior = resolve_mock_behavior(goal.constraints_json, self.mock_duration_sec)
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
        self._publish_status('starting', active=True, ready=False, error=False, detail='mock walk started', mission_id=mission_id)
        while not self.cancel_event.is_set():
            elapsed = time.monotonic() - start
            if outcome == 'timeout' and elapsed >= duration_sec:
                self._publish_status('walking', active=True, ready=False, error=False, detail='mock walk waiting', progress=0.95, mission_id=mission_id)
                time.sleep(0.2)
                continue
            if elapsed >= duration_sec:
                break
            progress = min(elapsed / max(duration_sec, 0.1), 0.95)
            self._publish_status('walking', active=True, ready=False, error=False, detail='mock walk running', progress=progress, mission_id=mission_id)
            time.sleep(0.2)
        if self.cancel_event.is_set():
            self._publish_status('stopped', active=False, ready=True, error=False, detail='mock walk stopped', mission_id=mission_id, clear_mission=True)
            return
        if outcome == 'fail':
            self._publish_status('failed', active=False, ready=False, error=True, detail='mock walk failed', error_code=ERROR_BACKEND_FAILED, mission_id=mission_id)
            return
        self._publish_status('completed', active=False, ready=True, error=False, detail='mock walk completed', progress=1.0, mission_id=mission_id)

    def _start_legacy(self, goal) -> tuple[bool, str, int]:
        try:
            service_module = importlib.import_module('planner_client.srv')
            set_goal_type = service_module.SetGoal
        except ImportError as exc:
            return False, f'planner_client unavailable: {exc}', ERROR_BACKEND_UNAVAILABLE

        client = self.create_client(set_goal_type, self.legacy_service_name, callback_group=self.callback_group)
        if not client.wait_for_service(timeout_sec=5.0):
            return False, 'legacy autowalk service unavailable', ERROR_BACKEND_UNAVAILABLE

        request = set_goal_type.Request()
        request.goal = goal.target_pose
        request.x = goal.target_pose.pose.position.x
        request.y = goal.target_pose.pose.position.y
        request.yaw = 0.0
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.05)
        response = future.result()
        if not getattr(response, 'success', False):
            return False, getattr(response, 'message', 'legacy autowalk rejected'), ERROR_BACKEND_FAILED

        self._publish_status('starting', active=True, ready=False, error=False, detail='legacy walk accepted', mission_id=goal.mission_id)
        return True, 'received', 0

    def _stop_mock(self) -> tuple[bool, str, int]:
        self.cancel_event.set()
        with self.lock:
            mission_id = self.current_mission_id
            running = self.worker_thread is not None and self.worker_thread.is_alive()
        if running:
            self._publish_status('stopping', active=True, ready=False, error=False, detail='mock walk stopping', mission_id=mission_id)
        else:
            self._publish_status('idle', active=False, ready=True, error=False, detail='mock walk idle', mission_id=mission_id, clear_mission=True)
        return True, 'received', 0

    def _stop_legacy(self) -> tuple[bool, str, int]:
        mission_id = self.current_mission_id
        if not mission_id:
            self._publish_status('idle', active=False, ready=True, error=False, detail='walk idle', mission_id='', clear_mission=True)
            return True, 'received', 0
        self._publish_status('stopping', active=True, ready=False, error=False, detail='legacy walk stopping', mission_id=mission_id)
        if not self._legacy_stop_client.wait_for_service(timeout_sec=2.0):
            return False, 'legacy walk stop service unavailable', ERROR_BACKEND_UNAVAILABLE
        future = self._legacy_stop_client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        response = future.result()
        if not response.success:
            return False, response.message or 'legacy walk stop rejected', ERROR_BACKEND_FAILED
        return True, 'received', 0

    def _on_legacy_status(self, msg: String) -> None:
        status = msg.data.strip().lower()
        phase_map = {
            'initialized': ('idle', False, True, False),
            'idle': ('idle', False, True, False),
            'goal_set': ('starting', True, False, False),
            'walking': ('walking', True, False, False),
            'success': ('completed', False, True, False),
            'stopped': ('stopped', False, True, False),
            'canceled': ('stopped', False, True, False),
            'aborted': ('failed', False, False, True),
            'failed': ('failed', False, False, True),
            'rejected': ('failed', False, False, True),
        }
        phase, active, ready, error = phase_map.get(status, ('walking', True, False, False))
        detail = f'legacy walk status={status}'
        error_code = ERROR_BACKEND_FAILED if error else 0
        clear_mission = phase in {'idle', 'stopped'}
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
        return self.current_phase not in {'idle', 'stopped'}

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

    @staticmethod
    def _build_result(accepted: bool, error_code: int, message: str) -> WalkMission.Result:
        result = WalkMission.Result()
        result.accepted = accepted
        result.error_code = error_code
        result.message = message
        return result


def main() -> None:
    rclpy.init()
    node = MobilityActionServer()
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
