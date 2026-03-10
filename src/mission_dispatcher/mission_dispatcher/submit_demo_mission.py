import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from integrated_mission_interfaces.srv import SubmitMission
from rclpy.node import Node


class DemoMissionClient(Node):
    def __init__(self) -> None:
        super().__init__('demo_mission_client')
        self.cli = self.create_client(SubmitMission, '/mission_dispatcher/submit_mission')

    def submit(self) -> int:
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('submit_mission service unavailable')
            return 1
        request = SubmitMission.Request()
        request.mission_id = f'demo-{int(time.time())}'
        request.walk_target = PoseStamped()
        request.walk_target.header.frame_id = 'map'
        request.walk_target.pose.position.x = 5.0
        request.walk_target.pose.position.y = 2.0
        request.walk_target.pose.orientation.w = 1.0
        request.dig_target_zone = 'bench-A'
        request.walk_constraints_json = '{"simulate": "success", "duration_sec": 3.0}'
        request.dig_parameters_json = '{"simulate": "success", "duration_sec": 4.0, "material_volume": 8.5}'
        request.priority = 1
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error('submit_mission request timed out')
            return 2
        response = future.result()
        if not response.accepted:
            self.get_logger().error(f'mission rejected: {response.message}')
            return 3
        self.get_logger().info(f'mission accepted: {response.message}')
        return 0


def main() -> None:
    rclpy.init(args=sys.argv)
    node = DemoMissionClient()
    try:
        raise SystemExit(node.submit())
    finally:
        node.destroy_node()
        rclpy.shutdown()
