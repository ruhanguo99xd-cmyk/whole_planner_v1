import importlib
import time

import rclpy
from integrated_mission_interfaces.action import DigMission
from integrated_mission_interfaces.msg import SubsystemStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from excavation_planner_core.mock_runtime import resolve_mock_behavior

ERROR_BACKEND_UNAVAILABLE = 2
ERROR_TIMEOUT = 3
ERROR_CANCELED = 4
ERROR_BACKEND_FAILED = 5


class ExcavationActionServer(Node):
    def __init__(self) -> None:
        super().__init__('excavation_planner_core')
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('mock_duration_sec', 5.0)
        self.declare_parameter('legacy_action_name', '/dig')
        self.backend = str(self.get_parameter('backend').value)
        self.mock_duration_sec = float(self.get_parameter('mock_duration_sec').value)
        self.legacy_action_name = str(self.get_parameter('legacy_action_name').value)
        self.status_pub = self.create_publisher(SubsystemStatus, '/excavation/status', 10)
        self.action_server = ActionServer(
            self,
            DigMission,
            '/excavation/execute',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info(f'excavation_planner_core started: backend={self.backend}')

    def goal_callback(self, goal_request) -> GoalResponse:
        if goal_request.timeout_sec <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self._publish_status(True, False, False, 'starting', goal.mission_id)
        if self.backend == 'mock':
            result = self._execute_mock(goal_handle, goal)
        elif self.backend == 'legacy_dig_action':
            result = self._execute_legacy(goal_handle, goal)
        else:
            goal_handle.abort()
            result = self._build_result(False, ERROR_BACKEND_UNAVAILABLE, f'unsupported backend: {self.backend}', False, 0.0)
        self._publish_status(False, result.success, not result.success, 'finished' if result.success else 'failed', result.message, result.error_code)
        return result

    def _execute_mock(self, goal_handle, goal) -> DigMission.Result:
        behavior = resolve_mock_behavior(goal.process_parameters_json, self.mock_duration_sec)
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            feedback = DigMission.Feedback()
            feedback.phase = 'digging'
            feedback.progress = min(elapsed / max(behavior.duration_sec, 0.1), 0.99)
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._build_result(False, ERROR_CANCELED, 'dig canceled', True, 0.0)
            if behavior.outcome == 'timeout' and elapsed > goal.timeout_sec:
                goal_handle.abort()
                return self._build_result(False, ERROR_TIMEOUT, 'dig mock timeout', True, 0.0)
            if elapsed >= behavior.duration_sec:
                break
            time.sleep(0.2)
        if behavior.outcome == 'fail':
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_FAILED, 'dig mock failure', True, behavior.material_volume)
        goal_handle.succeed()
        return self._build_result(True, 0, 'dig mock success', False, behavior.material_volume)

    def _execute_legacy(self, goal_handle, goal) -> DigMission.Result:
        try:
            action_module = importlib.import_module('shovel_interfaces.action')
            dig_action = action_module.Dig
        except ImportError as exc:
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_UNAVAILABLE, f'shovel_interfaces unavailable: {exc}', True, 0.0)
        client = ActionClient(self, dig_action, self.legacy_action_name, callback_group=self.callback_group)
        if not client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_UNAVAILABLE, 'legacy dig action unavailable', True, 0.0)
        legacy_goal = dig_action.Goal()
        legacy_goal.start = True
        send_future = client.send_goal_async(legacy_goal)
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._build_result(False, ERROR_CANCELED, 'dig canceled before dispatch', True, 0.0)
            time.sleep(0.1)
        legacy_goal_handle = send_future.result()
        if not legacy_goal_handle.accepted:
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_FAILED, 'legacy dig rejected', True, 0.0)
        result_future = legacy_goal_handle.get_result_async()
        start = time.monotonic()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                legacy_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return self._build_result(False, ERROR_CANCELED, 'dig canceled during legacy execution', True, 0.0)
            if time.monotonic() - start > goal.timeout_sec:
                legacy_goal_handle.cancel_goal_async()
                goal_handle.abort()
                return self._build_result(False, ERROR_TIMEOUT, 'legacy dig timeout', True, 0.0)
            feedback = DigMission.Feedback()
            feedback.phase = 'legacy_digging'
            feedback.progress = min((time.monotonic() - start) / max(goal.timeout_sec, 0.1), 0.95)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)
        wrapped = result_future.result()
        result = wrapped.result
        if getattr(result, 'code', 2) == 0:
            goal_handle.succeed()
            return self._build_result(True, 0, getattr(result, 'message', 'legacy dig success'), False, 0.0)
        goal_handle.abort()
        return self._build_result(False, ERROR_BACKEND_FAILED, getattr(result, 'message', 'legacy dig failure'), True, 0.0)

    def _publish_status(self, active: bool, ready: bool, error: bool, phase: str, detail: str, error_code: int = 0) -> None:
        msg = SubsystemStatus()
        msg.active = active
        msg.ready = ready
        msg.error = error
        msg.error_code = error_code
        msg.phase = phase
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    @staticmethod
    def _build_result(success: bool, error_code: int, message: str, retryable: bool, material_volume: float) -> DigMission.Result:
        result = DigMission.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        result.retryable = retryable
        result.material_volume = material_volume
        return result


def main() -> None:
    rclpy.init()
    node = ExcavationActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
