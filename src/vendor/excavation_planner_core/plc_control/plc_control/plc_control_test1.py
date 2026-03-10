import threading
import time
from typing import Optional

try:
    import snap7  # noqa: F401
    from snap7 import util  # noqa: F401
except ImportError:
    snap7 = None
    util = None

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class DigCommandNode(Node):
    def __init__(self):
        super().__init__('plc_control')
        self.start_pub = self.create_publisher(Bool, 'digging/perception_start', 10)
        self.excavation_start_pub = self.create_publisher(Bool, 'digging/excavation_start', 10)
        self.status_pub = self.create_publisher(String, '/dig/status', 10)
        self.finish_sub = self.create_subscription(Bool, 'digging/perception_finish', self.finish_callback, 10)
        self.start_srv = self.create_service(Trigger, '/dig/start', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/dig/stop', self.stop_callback)
        self.perception_finished = False
        self.cancel_event = threading.Event()
        self.worker_thread: Optional[threading.Thread] = None
        self.active = False
        self.current_phase = 'idle'
        self.lock = threading.Lock()
        self._publish_status('idle')
        self.get_logger().info('Dig command service started')

    def finish_callback(self, msg: Bool) -> None:
        if msg.data:
            self.perception_finished = True
            self.get_logger().info('Perception finished signal received')

    def start_callback(self, request, response):
        del request
        with self.lock:
            if self._worker_running_locked():
                response.success = False
                response.message = 'dig command busy'
                return response
            self.perception_finished = False
            self.cancel_event = threading.Event()
            self.active = True
            self.current_phase = 'starting'
            self._publish_status('starting')
            self.worker_thread = threading.Thread(target=self._run_sequence, daemon=True)
            self.worker_thread.start()
        response.success = True
        response.message = 'received'
        return response

    def stop_callback(self, request, response):
        del request
        self.cancel_event.set()
        with self.lock:
            if self._worker_running_locked():
                self.current_phase = 'stopping'
                self._publish_status('stopping')
            else:
                self.active = False
                self.current_phase = 'idle'
                self._publish_status('idle')
        response.success = True
        response.message = 'received'
        return response

    def publish_start_signal(self) -> None:
        start_msg = Bool()
        start_msg.data = True
        self.start_pub.publish(start_msg)
        self.get_logger().info('Published perception start signal')

    def publish_excavation_start(self) -> None:
        start_msg = Bool()
        start_msg.data = True
        self.excavation_start_pub.publish(start_msg)
        self.get_logger().info('Published excavation start signal')

    def _run_sequence(self) -> None:
        try:
            self._sleep_or_cancel('rotating_scan', 0.5)
            self.publish_start_signal()
            self._publish_status('waiting_perception')
            while not self.perception_finished:
                self._check_cancel()
                time.sleep(0.1)
            self.perception_finished = False
            self._sleep_or_cancel('resetting', 0.5)
            self._publish_status('digging')
            for _ in range(25):
                self._check_cancel()
                time.sleep(0.2)
            self.publish_excavation_start()
            with self.lock:
                self.active = False
                self.current_phase = 'completed'
            self._publish_status('completed')
        except RuntimeError:
            with self.lock:
                self.active = False
                self.current_phase = 'stopped'
            self._publish_status('stopped')
        except Exception as exc:
            with self.lock:
                self.active = False
                self.current_phase = 'failed'
            self._publish_status('failed')
            self.get_logger().error(f'Dig command failed: {exc}')

    def _sleep_or_cancel(self, phase: str, duration_sec: float) -> None:
        self._publish_status(phase)
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self._check_cancel()
            time.sleep(0.1)

    def _check_cancel(self) -> None:
        if self.cancel_event.is_set():
            raise RuntimeError('dig command canceled')

    def _worker_running_locked(self) -> bool:
        return self.worker_thread is not None and self.worker_thread.is_alive()

    def _publish_status(self, phase: str) -> None:
        msg = String()
        msg.data = phase
        self.status_pub.publish(msg)
        self.get_logger().info(f'Dig status -> {phase}')


def main():
    rclpy.init()
    node = DigCommandNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f'Operation failed: {exc}')
    finally:
        node.cancel_event.set()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
