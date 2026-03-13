import sys
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
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
        request.use_material_target = True
        request.current_pose = PoseStamped()
        request.current_pose.header.frame_id = 'map'
        request.current_pose.pose.position.x = 0.0
        request.current_pose.pose.position.y = 0.0
        request.current_pose.pose.orientation.w = 1.0
        request.walk_target = PoseStamped()
        request.walk_target.header.frame_id = 'map'
        request.walk_target.pose.orientation.w = 1.0
        request.material_reference_pose = PoseStamped()
        request.material_reference_pose.header.frame_id = 'map'
        request.material_reference_pose.pose.position.x = 10.0
        request.material_reference_pose.pose.position.y = 0.0
        request.material_reference_pose.pose.orientation.w = 1.0
        request.material_outline = [
            Point(x=8.0, y=0.0, z=0.0),
            Point(x=10.0, y=0.0, z=0.0),
            Point(x=12.0, y=0.0, z=0.0),
        ]
        request.desired_standoff_m = 3.0
        request.material_profile_json = '{"material_kind": "ore", "target_strategy": "auto"}'
        request.target_planner_constraints_json = '{"edge_near_threshold_m": 1.5}'
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
        self.get_logger().info(
            'mission accepted: %s resolved_target=(%.3f, %.3f) detail=%s'
            % (
                response.message,
                response.resolved_walk_target.pose.position.x,
                response.resolved_walk_target.pose.position.y,
                response.resolution_detail,
            )
        )
        return 0


def main() -> None:
    rclpy.init(args=sys.argv)
    node = DemoMissionClient()
    try:
        raise SystemExit(node.submit())
    finally:
        node.destroy_node()
        rclpy.shutdown()
