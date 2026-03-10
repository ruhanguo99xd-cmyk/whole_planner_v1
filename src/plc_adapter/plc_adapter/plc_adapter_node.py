import importlib
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from integrated_mission_interfaces.msg import PlcSnapshot
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PlcAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('plc_adapter')
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mock_sequence_file', '')
        self.declare_parameter('plc_ip', '192.168.2.20')
        self.declare_parameter('rack', 0)
        self.declare_parameter('slot', 1)
        self.backend = str(self.get_parameter('backend').value)
        self.publisher = self.create_publisher(PlcSnapshot, '/plc/status', 10)
        self._sequence: List[Dict[str, Any]] = []
        self._sequence_loop = True
        self._step_index = 0
        self._step_started_at = time.monotonic()
        self._snap7 = None
        self._snap7_client = None

        if self.backend == 'mock':
            self._load_mock_sequence()
        else:
            self._init_real_backend()

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._publish_snapshot)
        self.get_logger().info(f'plc_adapter started: backend={self.backend}')

    def _load_mock_sequence(self) -> None:
        sequence_file = str(self.get_parameter('mock_sequence_file').value)
        if not sequence_file:
            raise RuntimeError('mock_sequence_file must be configured for mock backend')
        path = Path(sequence_file)
        yaml = importlib.import_module('yaml')
        with path.open('r', encoding='utf-8') as handle:
            payload = yaml.safe_load(handle)
        self._sequence = payload.get('steps', [])
        self._sequence_loop = bool(payload.get('loop', True))
        if not self._sequence:
            raise RuntimeError(f'No steps configured in {path}')
        self._step_started_at = time.monotonic()

    def _init_real_backend(self) -> None:
        try:
            self._snap7 = importlib.import_module('snap7')
        except ImportError as exc:
            raise RuntimeError('snap7 not available for real PLC backend') from exc
        self._snap7_client = self._snap7.client.Client()
        self._snap7_client.connect(
            str(self.get_parameter('plc_ip').value),
            int(self.get_parameter('rack').value),
            int(self.get_parameter('slot').value),
        )

    def _publish_snapshot(self) -> None:
        snapshot = self._read_mock_snapshot() if self.backend == 'mock' else self._read_real_snapshot()
        snapshot.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(snapshot)

    def _read_mock_snapshot(self) -> PlcSnapshot:
        now = time.monotonic()
        current = self._sequence[self._step_index]
        duration = float(current.get('duration_sec', 1.0))
        if now - self._step_started_at >= duration:
            if self._step_index + 1 < len(self._sequence):
                self._step_index += 1
            elif self._sequence_loop:
                self._step_index = 0
            self._step_started_at = now
            current = self._sequence[self._step_index]
        return self._step_to_msg(current)

    def _read_real_snapshot(self) -> PlcSnapshot:
        msg = PlcSnapshot()
        msg.machine_ready = self._read_q_bit(1, 0)
        msg.safe_to_walk = self._read_q_bit(1, 1)
        msg.safe_to_dig = self._read_q_bit(1, 2)
        msg.walking_requested = self._read_q_bit(1, 3)
        msg.digging_requested = self._read_q_bit(1, 4)
        msg.fault_active = self._read_q_bit(1, 5)
        msg.manual_override = self._read_q_bit(1, 6)
        msg.fault_code = 1 if msg.fault_active else 0
        msg.source = 'real_plc'
        return msg

    def _read_q_bit(self, byte: int, bit: int) -> bool:
        data = self._snap7_client.read_area(self._snap7.types.Areas.PA, 0, byte, 1)
        return bool(data[0] & (1 << bit))

    @staticmethod
    def _step_to_msg(step: Dict[str, Any]) -> PlcSnapshot:
        msg = PlcSnapshot()
        msg.machine_ready = bool(step.get('machine_ready', False))
        msg.safe_to_walk = bool(step.get('safe_to_walk', False))
        msg.safe_to_dig = bool(step.get('safe_to_dig', False))
        msg.walking_requested = bool(step.get('walking_requested', False))
        msg.digging_requested = bool(step.get('digging_requested', False))
        msg.fault_active = bool(step.get('fault_active', False))
        msg.manual_override = bool(step.get('manual_override', False))
        msg.fault_code = int(step.get('fault_code', 0))
        msg.source = str(step.get('source', 'mock'))
        return msg


def main() -> None:
    rclpy.init()
    node = PlcAdapterNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
