import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MockPointcloudServer(Node):
    def __init__(self) -> None:
        super().__init__('mock_pointcloud_server')
        srv_module = __import__('point_cloud_processing_pkg.srv', fromlist=['Wuliaoprocess'])
        srv_type = srv_module.Wuliaoprocess
        self.server = self.create_service(srv_type, 'process_pointcloud', self.handle_request)
        self.get_logger().info('mock_pointcloud_server started')

    def handle_request(self, request, response):
        del request
        response.issuccess = True
        response.prs_coefficients = [0.1] * 28
        response.bounding_boxes = [0.0, 10.0, 0.0, 10.0, 0.0, 5.0]
        return response


def main() -> None:
    rclpy.init()
    node = MockPointcloudServer()
    executor = MultiThreadedExecutor(num_threads=2)
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
