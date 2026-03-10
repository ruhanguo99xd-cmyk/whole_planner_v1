import importlib
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

from mobility_planner_core.mock_runtime import resolve_mock_behavior

ERROR_BACKEND_UNAVAILABLE = 2
ERROR_TIMEOUT = 3
ERROR_CANCELED = 4
ERROR_BACKEND_FAILED = 5


class MobilityActionServer(Node):
    def __init__(self) -> None:
        super().__init__('mobility_planner_core')
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('mock_duration_sec', 4.0)
        self.declare_parameter('legacy_service_name', '/autonomous_walk/set_goal')
        self.declare_parameter('legacy_status_topic', '/autonomous_walk/status')
        self.backend = str(self.get_parameter('backend').value)
        self.mock_duration_sec = float(self.get_parameter('mock_duration_sec').value)
        self.legacy_service_name = str(self.get_parameter('legacy_service_name').value)
        self.legacy_status_topic = str(self.get_parameter('legacy_status_topic').value)
        self.status_pub = self.create_publisher(SubsystemStatus, '/mobility/status', 10)
        self.legacy_status: str = 'idle'
        self.create_subscription(String, self.legacy_status_topic, self._on_legacy_status, 10)
        self.action_server = ActionServer(
            self,
            WalkMission,
            '/mobility/execute',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info(f'mobility_planner_core started: backend={self.backend}')

    def _on_legacy_status(self, msg: String) -> None:
        self.legacy_status = msg.data

    def goal_callback(self, goal_request) -> GoalResponse:
        if goal_request.timeout_sec <= 0.0:
            self.get_logger().warning('Reject walk goal with invalid timeout')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self._publish_status(active=True, ready=False, error=False, phase='starting', detail=goal.mission_id)
        if self.backend == 'mock':
            result = self._execute_mock(goal_handle, goal)
        elif self.backend == 'legacy_autonomous_walk_service':
            result = self._execute_legacy(goal_handle, goal)
        else:
            result = WalkMission.Result()
            result.success = False
            result.error_code = ERROR_BACKEND_UNAVAILABLE
            result.message = f'unsupported backend: {self.backend}'
            result.retryable = False
            goal_handle.abort()
        self._publish_status(
            active=False,
            ready=result.success,
            error=not result.success,
            phase='finished' if result.success else 'failed',
            detail=result.message,
            error_code=result.error_code,
        )
        return result

    def _execute_mock(self, goal_handle, goal) -> WalkMission.Result:
        behavior = resolve_mock_behavior(goal.constraints_json, self.mock_duration_sec)
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            progress = min(elapsed / max(behavior.duration_sec, 0.1), 0.99)
            feedback = WalkMission.Feedback()
            feedback.phase = 'walking'
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._build_result(False, ERROR_CANCELED, 'walk canceled', retryable=True)
            if behavior.outcome == 'timeout' and elapsed > goal.timeout_sec:
                goal_handle.abort()
                return self._build_result(False, ERROR_TIMEOUT, 'walk mock timeout', retryable=True)
            if elapsed >= behavior.duration_sec:
                break
            time.sleep(0.2)
        if behavior.outcome == 'fail':
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_FAILED, 'walk mock failure', retryable=True)
        goal_handle.succeed()
        return self._build_result(True, 0, 'walk mock success', retryable=False)

    def _execute_legacy(self, goal_handle, goal) -> WalkMission.Result:
        try:
            service_module = importlib.import_module('planner_client.srv')
            set_goal_type = service_module.SetGoal
        except ImportError as exc:
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_UNAVAILABLE, f'planner_client unavailable: {exc}', retryable=True)
        client = self.create_client(set_goal_type, self.legacy_service_name, callback_group=self.callback_group)
        if not client.wait_for_service(timeout_sec=5.0):
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_UNAVAILABLE, 'legacy autowalk service unavailable', retryable=True)
        request = set_goal_type.Request()
        request.goal = goal.target_pose
        request.x = goal.target_pose.pose.position.x
        request.y = goal.target_pose.pose.position.y
        request.yaw = 0.0
        future = client.call_async(request)
        while not future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._build_result(False, ERROR_CANCELED, 'walk canceled before dispatch', retryable=True)
            time.sleep(0.1)
        response = future.result()
        if not getattr(response, 'success', False):
            goal_handle.abort()
            return self._build_result(False, ERROR_BACKEND_FAILED, getattr(response, 'message', 'legacy autowalk rejected'), retryable=True)
        self.legacy_status = 'goal_dispatched'
        start = time.monotonic()
        while time.monotonic() - start <= goal.timeout_sec:
            feedback = WalkMission.Feedback()
            feedback.phase = self.legacy_status
            feedback.progress = min((time.monotonic() - start) / max(goal.timeout_sec, 0.1), 0.95)
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._build_result(False, ERROR_CANCELED, 'walk canceled during legacy execution', retryable=True)
            if self.legacy_status == 'success':
                goal_handle.succeed()
                return self._build_result(True, 0, 'legacy autowalk success', retryable=False)
            if self.legacy_status in {'aborted', 'failed', 'rejected'}:
                goal_handle.abort()
                return self._build_result(False, ERROR_BACKEND_FAILED, f'legacy autowalk status={self.legacy_status}', retryable=True)
            time.sleep(0.2)
        goal_handle.abort()
        return self._build_result(False, ERROR_TIMEOUT, 'legacy autowalk timeout', retryable=True)

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
    def _build_result(success: bool, error_code: int, message: str, retryable: bool) -> WalkMission.Result:
        result = WalkMission.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        result.retryable = retryable
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
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
