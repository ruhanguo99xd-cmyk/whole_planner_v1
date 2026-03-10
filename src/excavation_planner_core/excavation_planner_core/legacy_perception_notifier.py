import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


class LegacyPerceptionNotifier(Node):
    def __init__(self) -> None:
        super().__init__('legacy_perception_notifier')
        self.finish_pub = self.create_publisher(Bool, 'digging/perception_finish', 10)
        self.create_subscription(Bool, 'digging/perception_start', self._on_start, 10)
        self.declare_parameter('delay_sec', 1.0)
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.get_logger().info('legacy_perception_notifier started')

    def _on_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        threading.Thread(target=self._publish_later, daemon=True).start()

    def _publish_later(self) -> None:
        time.sleep(self.delay_sec)
        done = Bool()
        done.data = True
        self.finish_pub.publish(done)
        self.get_logger().info('Published digging/perception_finish from legacy notifier')


def main() -> None:
    rclpy.init()
    node = LegacyPerceptionNotifier()
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
