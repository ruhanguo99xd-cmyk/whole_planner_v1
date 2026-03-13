from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import snap7
except Exception:  # pragma: no cover
    snap7 = None


@dataclass(frozen=True)
class S7Address:
    db_number: int
    byte_offset: int


def parse_db_address(addr: str) -> S7Address:
    value = addr.strip().lower().replace(' ', '')
    if value.startswith('db') and '.dbd' in value:
        left, right = value.split('.dbd', 1)
        return S7Address(db_number=int(left[2:]), byte_offset=int(right))
    if value.startswith('db') and '.' in value:
        left, right = value.split('.', 1)
        return S7Address(db_number=int(left[2:]), byte_offset=int(right))
    raise ValueError(f'Unsupported db_address format: {addr}')


class LiteSlamSwingAngleBridge(Node):
    def __init__(self) -> None:
        super().__init__('lite_slam_swing_angle_bridge')
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('publish_topic', '/lite_slam/swing_angle_deg')
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('mock_rotation_speed_deg_per_sec', 18.0)
        self.declare_parameter('endpoint', '192.168.1.10')
        self.declare_parameter('rack', 0)
        self.declare_parameter('slot', 1)
        self.declare_parameter('db_address', 'db9999.9999')
        self.declare_parameter('read_size_bytes', 4)
        self.declare_parameter('signed_ticks', False)
        self.declare_parameter('resolution_ticks_per_rev', 16384)
        self.declare_parameter('invert_direction', False)

        self.backend = str(self.get_parameter('backend').value)
        self.publish_topic = str(self.get_parameter('publish_topic').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.mock_rotation_speed_deg_per_sec = float(self.get_parameter('mock_rotation_speed_deg_per_sec').value)
        self.endpoint = str(self.get_parameter('endpoint').value)
        self.rack = int(self.get_parameter('rack').value)
        self.slot = int(self.get_parameter('slot').value)
        self.db_address = str(self.get_parameter('db_address').value)
        self.read_size_bytes = int(self.get_parameter('read_size_bytes').value)
        self.signed_ticks = bool(self.get_parameter('signed_ticks').value)
        self.resolution_ticks_per_rev = int(self.get_parameter('resolution_ticks_per_rev').value)
        self.invert_direction = bool(self.get_parameter('invert_direction').value)

        self.publisher = self.create_publisher(Float32, self.publish_topic, 10)
        self._address = parse_db_address(self.db_address)
        self._plc_client: Optional[object] = None
        self._mock_start = time.monotonic()
        self._timer = self.create_timer(self.publish_period_sec, self._publish_angle)
        self.get_logger().info(
            f'lite_slam_swing_angle_bridge started: backend={self.backend} topic={self.publish_topic}'
        )

    def _publish_angle(self) -> None:
        try:
            angle_deg = self._read_angle_deg()
        except Exception as exc:
            self.get_logger().error(f'Failed to read swing angle: {exc}')
            return
        msg = Float32()
        msg.data = float(angle_deg)
        self.publisher.publish(msg)

    def _read_angle_deg(self) -> float:
        if self.backend == 'mock':
            elapsed = time.monotonic() - self._mock_start
            raw = elapsed * self.mock_rotation_speed_deg_per_sec
            return float(raw % 360.0)
        if self.backend == 'plc_s7':
            ticks = self._read_ticks_from_plc()
            angle_deg = (ticks / max(self.resolution_ticks_per_rev, 1)) * 360.0
            if self.invert_direction:
                angle_deg = (-angle_deg) % 360.0
            return float(angle_deg % 360.0)
        raise RuntimeError(f'Unsupported backend: {self.backend}')

    def _ensure_plc_connected(self) -> None:
        if snap7 is None:
            raise RuntimeError('python-snap7 not installed')
        if self._plc_client is None:
            self._plc_client = snap7.client.Client()
        if not self._plc_client.get_connected():
            self._plc_client.connect(self.endpoint, self.rack, self.slot)

    def _read_ticks_from_plc(self) -> int:
        self._ensure_plc_connected()
        raw = self._plc_client.db_read(self._address.db_number, self._address.byte_offset, self.read_size_bytes)
        ticks = int.from_bytes(raw, byteorder='big', signed=self.signed_ticks)
        return ticks % max(self.resolution_ticks_per_rev, 1)


def main() -> None:
    rclpy.init()
    node = LiteSlamSwingAngleBridge()
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
