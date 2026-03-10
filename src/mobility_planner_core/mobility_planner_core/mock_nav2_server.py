import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MockNavigateToPoseServer(Node):
    def __init__(self) -> None:
        super().__init__('mock_nav2_server')
        nav2_action = __import__('nav2_msgs.action', fromlist=['NavigateToPose']).NavigateToPose
        self._action_type = nav2_action
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('duration_sec', 3.0)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.server = ActionServer(
            self,
            self._action_type,
            str(self.get_parameter('action_name').value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('mock_nav2_server started')

    def goal_callback(self, goal_request) -> GoalResponse:
        del goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback_type = self._action_type.Feedback
        result_type = self._action_type.Result
        start = time.monotonic()
        while time.monotonic() - start < self.duration_sec:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result_type()
            feedback = feedback_type()
            feedback.distance_remaining = max(self.duration_sec - (time.monotonic() - start), 0.0)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)
        goal_handle.succeed()
        return result_type()


def main() -> None:
    rclpy.init()
    node = MockNavigateToPoseServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
