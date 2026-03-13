from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk
from typing import Any

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, PoseStamped, PolygonStamped, Twist
from integrated_mission_interfaces.action import DigMission, WalkMission
from integrated_mission_interfaces.msg import PlannerMode, PlcSnapshot, SubsystemStatus
from integrated_mission_interfaces.srv import SubmitMission
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

from mission_operator_hmi.helpers import (
    ViewTransform,
    compute_view_transform,
    compute_bounds,
    downsample_points,
    extract_occupancy_points,
    normalize_json_text,
    parse_outline_text,
    project_points,
    project_points_with_transform,
    project_series,
    screen_to_world,
)

MODE_NAMES = {
    PlannerMode.IDLE: 'IDLE',
    PlannerMode.WALK_PREP: 'WALK_PREP',
    PlannerMode.WALKING: 'WALKING',
    PlannerMode.DIG_PREP: 'DIG_PREP',
    PlannerMode.DIGGING: 'DIGGING',
    PlannerMode.TRANSITION: 'TRANSITION',
    PlannerMode.FAULT: 'FAULT',
}

COMMAND_START = 1
COMMAND_STOP = 2
COMMAND_CANCEL = 3


@dataclass
class DigDebugState:
    seg1: list[tuple[float, float, float]]
    seg2: list[tuple[float, float, float]]
    seg3: list[tuple[float, float, float]]
    optimization_candidate: list[tuple[float, float, float]]
    optimization_metrics: list[float]
    time_axis: list[float]
    vgan_result: list[float]
    vrope_result: list[float]
    gan_len: list[float]
    rope_len: list[float]
    load_rotation_deg: list[float]
    return_rotation_deg: list[float]
    summary: str


class MissionOperatorBackend(Node):
    def __init__(self) -> None:
        super().__init__('integrated_operator_hmi')
        self.lock = threading.RLock()
        self.track_width_m = float(self.declare_parameter('track_width_m', 1.925).value)
        self.mode: dict[str, Any] = {'name': 'IDLE', 'mission_id': '', 'reason': 'startup'}
        self.walk_status: dict[str, Any] = self._empty_status()
        self.dig_status: dict[str, Any] = self._empty_status()
        self.plc_status: dict[str, Any] = {
            'machine_ready': False,
            'safe_to_walk': False,
            'safe_to_dig': False,
            'fault_active': False,
            'manual_override': False,
            'fault_code': 0,
            'source': '--',
        }
        self.pose_source = '--'
        self.current_pose: dict[str, Any] | None = None
        self.goal_pose: tuple[float, float, float] | None = None
        self.auto_goal_pose: tuple[float, float, float] | None = None
        self.plan_points: list[tuple[float, float]] = []
        self.material_boundary: list[tuple[float, float]] = []
        self.map_points: list[tuple[float, float]] = []
        self.global_costmap_points: list[tuple[float, float]] = []
        self.local_costmap_points: list[tuple[float, float]] = []
        self.cmd_vel: dict[str, float] = {
            'linear': 0.0,
            'angular': 0.0,
            'left_track': 0.0,
            'right_track': 0.0,
        }
        self.track_history: deque[tuple[float, float, float]] = deque(maxlen=600)
        self.dig_debug = DigDebugState([], [], [], [], [], [], [], [], [], [], [], [], '等待实时轨迹 topic')
        self.swing_angle_deg = 0.0
        self.events: deque[str] = deque(maxlen=200)
        self._latest_pose_wall_time = 0.0
        self._latest_cmd_vel_wall_time = 0.0

        self.start_cli = self.create_client(Trigger, '/mission_dispatcher/start')
        self.stop_cli = self.create_client(Trigger, '/mission_dispatcher/stop')
        self.recover_cli = self.create_client(Trigger, '/mission_dispatcher/recover')
        self.submit_cli = self.create_client(SubmitMission, '/mission_dispatcher/submit_mission')
        self.walk_cli = ActionClient(self, WalkMission, '/mobility/execute')
        self.dig_cli = ActionClient(self, DigMission, '/excavation/execute')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(PlannerMode, '/mission_dispatcher/mode', self._on_mode, 10)
        self.create_subscription(SubsystemStatus, '/mobility/status', self._on_walk_status, 10)
        self.create_subscription(SubsystemStatus, '/excavation/status', self._on_dig_status, 10)
        self.create_subscription(PlcSnapshot, '/plc/status', self._on_plc_status, 10)
        self.create_subscription(RosPath, '/plan', self._on_plan, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 2)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_global_costmap, 2)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_local_costmap, 2)
        self.create_subscription(PolygonStamped, '/mobility/material_boundary', self._on_material_boundary, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.create_subscription(PoseStamped, '/auto_goal_pose', self._on_auto_goal_pose, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Float32, '/lite_slam/swing_angle_deg', self._on_swing_angle, 10)
        self.create_subscription(RosPath, '/digging/debug/segment1_path', self._on_dig_segment1, transient_qos)
        self.create_subscription(RosPath, '/digging/debug/segment2_path', self._on_dig_segment2, transient_qos)
        self.create_subscription(RosPath, '/digging/debug/segment3_path', self._on_dig_segment3, transient_qos)
        self.create_subscription(RosPath, '/digging/debug/optimization_candidate_path', self._on_dig_optimization_candidate, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/optimization_metrics', self._on_dig_optimization_metrics, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/time_axis', self._on_dig_time_axis, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/vgan_result', self._on_dig_vgan, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/vrope_result', self._on_dig_vrope, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/gan_len', self._on_dig_gan_len, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/rope_len', self._on_dig_rope_len, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/load_rotation_deg', self._on_load_rotation, transient_qos)
        self.create_subscription(Float32MultiArray, '/digging/debug/return_rotation_deg', self._on_return_rotation, transient_qos)
        for topic_name, source_name in (
            ('/ekf_odom', 'ekf_odom'),
            ('/odom', 'odom'),
            ('/fake_odom', 'fake_odom'),
            ('/gps_odom', 'gps_odom'),
        ):
            self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, source_name=source_name: self._on_odom(msg, source_name),
                10,
            )

        self._append_event('系统', '统一上位机已连接 ROS2 数据通道')

    @staticmethod
    def _empty_status() -> dict[str, Any]:
        return {
            'mission_id': '',
            'phase': 'idle',
            'detail': '--',
            'active': False,
            'ready': False,
            'error': False,
            'error_code': 0,
            'progress': 0.0,
        }

    def _append_event(self, source: str, message: str) -> None:
        timestamp = time.strftime('%H:%M:%S')
        with self.lock:
            self.events.appendleft(f'[{timestamp}] {source}: {message}')

    def _on_mode(self, msg: PlannerMode) -> None:
        with self.lock:
            self.mode = {
                'name': MODE_NAMES.get(msg.mode, str(msg.mode)),
                'mission_id': msg.mission_id,
                'reason': msg.reason,
            }
        self._append_event('大规划', f"{MODE_NAMES.get(msg.mode, msg.mode)} / {msg.reason}")

    def _on_walk_status(self, msg: SubsystemStatus) -> None:
        with self.lock:
            self.walk_status = self._status_to_dict(msg)
        self._append_event('行走', f"{msg.phase} / {msg.detail}")

    def _on_dig_status(self, msg: SubsystemStatus) -> None:
        with self.lock:
            self.dig_status = self._status_to_dict(msg)
        self._append_event('挖掘', f"{msg.phase} / {msg.detail}")

    def _on_plc_status(self, msg: PlcSnapshot) -> None:
        with self.lock:
            self.plc_status = {
                'machine_ready': msg.machine_ready,
                'safe_to_walk': msg.safe_to_walk,
                'safe_to_dig': msg.safe_to_dig,
                'fault_active': msg.fault_active,
                'manual_override': msg.manual_override,
                'fault_code': int(msg.fault_code),
                'source': msg.source,
            }

    def _on_plan(self, msg: RosPath) -> None:
        with self.lock:
            self.plan_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._store_grid_layer(msg, 'map')

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._store_grid_layer(msg, 'global_costmap')

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._store_grid_layer(msg, 'local_costmap')

    def _on_material_boundary(self, msg: PolygonStamped) -> None:
        with self.lock:
            self.material_boundary = [(point.x, point.y) for point in msg.polygon.points]

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            self.goal_pose = self._pose_tuple(msg)

    def _on_auto_goal_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            self.auto_goal_pose = self._pose_tuple(msg)

    def _on_swing_angle(self, msg: Float32) -> None:
        with self.lock:
            self.swing_angle_deg = float(msg.data)

    def _on_cmd_vel(self, msg: Twist) -> None:
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        left_track = linear - angular * self.track_width_m * 0.5
        right_track = linear + angular * self.track_width_m * 0.5
        now = time.monotonic()
        with self.lock:
            self.cmd_vel = {
                'linear': linear,
                'angular': angular,
                'left_track': left_track,
                'right_track': right_track,
            }
            self._latest_cmd_vel_wall_time = now
            self.track_history.append((now, left_track, right_track))

    def _store_grid_layer(self, msg: OccupancyGrid, layer_name: str) -> None:
        threshold = 50 if layer_name == 'map' else 10
        points = extract_occupancy_points(
            msg.data,
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            threshold=threshold,
            max_points=4500 if layer_name == 'map' else 2500,
        )
        with self.lock:
            if layer_name == 'map':
                self.map_points = points
            elif layer_name == 'global_costmap':
                self.global_costmap_points = points
            else:
                self.local_costmap_points = points

    @staticmethod
    def _path_points(msg: RosPath) -> list[tuple[float, float, float]]:
        return [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in msg.poses
        ]

    def _store_dig_series(self, key: str, msg: Float32MultiArray) -> None:
        values = [float(value) for value in msg.data]
        with self.lock:
            setattr(self.dig_debug, key, values)
            self.dig_debug.summary = '实时调试 topic 已连接'

    def _on_dig_segment1(self, msg: RosPath) -> None:
        with self.lock:
            self.dig_debug.seg1 = self._path_points(msg)
            self.dig_debug.summary = '实时调试 topic 已连接'

    def _on_dig_segment2(self, msg: RosPath) -> None:
        with self.lock:
            self.dig_debug.seg2 = self._path_points(msg)
            self.dig_debug.summary = '实时调试 topic 已连接'

    def _on_dig_segment3(self, msg: RosPath) -> None:
        with self.lock:
            self.dig_debug.seg3 = self._path_points(msg)
            self.dig_debug.summary = '实时调试 topic 已连接'

    def _on_dig_optimization_candidate(self, msg: RosPath) -> None:
        with self.lock:
            self.dig_debug.optimization_candidate = self._path_points(msg)
            self.dig_debug.summary = '优化中间态 topic 已连接'

    def _on_dig_optimization_metrics(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('optimization_metrics', msg)

    def _on_dig_time_axis(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('time_axis', msg)

    def _on_dig_vgan(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('vgan_result', msg)

    def _on_dig_vrope(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('vrope_result', msg)

    def _on_dig_gan_len(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('gan_len', msg)

    def _on_dig_rope_len(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('rope_len', msg)

    def _on_load_rotation(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('load_rotation_deg', msg)

    def _on_return_rotation(self, msg: Float32MultiArray) -> None:
        self._store_dig_series('return_rotation_deg', msg)

    def _on_odom(self, msg: Odometry, source_name: str) -> None:
        now = time.monotonic()
        yaw = self._yaw_deg(msg.pose.pose.orientation)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        with self.lock:
            if now >= self._latest_pose_wall_time:
                self._latest_pose_wall_time = now
                self.pose_source = source_name
                self.current_pose = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'yaw_deg': yaw,
                    'speed_mps': speed,
                }

    @staticmethod
    def _status_to_dict(msg: SubsystemStatus) -> dict[str, Any]:
        return {
            'mission_id': msg.mission_id,
            'phase': msg.phase,
            'detail': msg.detail,
            'active': msg.active,
            'ready': msg.ready,
            'error': msg.error,
            'error_code': int(msg.error_code),
            'progress': float(msg.progress),
        }

    @staticmethod
    def _yaw_deg(orientation: Any) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _pose_tuple(self, msg: PoseStamped) -> tuple[float, float, float]:
        return (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            self._yaw_deg(msg.pose.orientation),
        )

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                'mode': dict(self.mode),
                'walk_status': dict(self.walk_status),
                'dig_status': dict(self.dig_status),
                'plc_status': dict(self.plc_status),
                'pose_source': self.pose_source,
                'current_pose': dict(self.current_pose) if self.current_pose else None,
                'goal_pose': tuple(self.goal_pose) if self.goal_pose else None,
                'auto_goal_pose': tuple(self.auto_goal_pose) if self.auto_goal_pose else None,
                'plan_points': list(self.plan_points),
                'material_boundary': list(self.material_boundary),
                'map_points': list(self.map_points),
                'global_costmap_points': list(self.global_costmap_points),
                'local_costmap_points': list(self.local_costmap_points),
                'cmd_vel': dict(self.cmd_vel),
                'track_history': list(self.track_history),
                'dig_debug': DigDebugState(
                    list(self.dig_debug.seg1),
                    list(self.dig_debug.seg2),
                    list(self.dig_debug.seg3),
                    list(self.dig_debug.optimization_candidate),
                    list(self.dig_debug.optimization_metrics),
                    list(self.dig_debug.time_axis),
                    list(self.dig_debug.vgan_result),
                    list(self.dig_debug.vrope_result),
                    list(self.dig_debug.gan_len),
                    list(self.dig_debug.rope_len),
                    list(self.dig_debug.load_rotation_deg),
                    list(self.dig_debug.return_rotation_deg),
                    self.dig_debug.summary,
                ),
                'swing_angle_deg': float(self.swing_angle_deg),
                'events': list(self.events),
            }

    def publish_goal(self, x_value: float, y_value: float, yaw_deg: float) -> None:
        msg = self._make_pose(x_value, y_value, yaw_deg)
        self.goal_pub.publish(msg)
        self._append_event('行走', f'手动目标已发布: ({x_value:.2f}, {y_value:.2f}, {yaw_deg:.1f}deg)')

    def request_start(self) -> None:
        self._call_trigger('开始自动任务', self.start_cli)

    def request_stop(self) -> None:
        self._call_trigger('停止自动任务', self.stop_cli)

    def request_recover(self) -> None:
        self._call_trigger('故障恢复', self.recover_cli)

    def _call_trigger(self, label: str, client: Any) -> None:
        def worker() -> None:
            if not client.wait_for_service(timeout_sec=2.0):
                self._append_event('HMI', f'{label}失败：service 不可用')
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda fut: self._on_trigger_done(label, fut))

        threading.Thread(target=worker, daemon=True).start()

    def _on_trigger_done(self, label: str, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self._append_event('HMI', f'{label}失败：{exc}')
            return
        result_text = '成功' if response.success else '失败'
        self._append_event('HMI', f'{label}{result_text}：{response.message}')

    def request_submit_mission(self, payload: dict[str, Any]) -> None:
        def worker() -> None:
            if not self.submit_cli.wait_for_service(timeout_sec=2.0):
                self._append_event('HMI', '提交任务失败：submit_mission service 不可用')
                return
            request = self._build_submit_request(payload)
            future = self.submit_cli.call_async(request)
            future.add_done_callback(self._on_submit_done)

        threading.Thread(target=worker, daemon=True).start()

    def request_manual_walk_start(
        self,
        *,
        x_value: float,
        y_value: float,
        yaw_deg: float,
        constraints_json: str,
        mission_id: str,
        priority: int,
        timeout_sec: float = 20.0,
    ) -> None:
        goal = WalkMission.Goal()
        goal.command = COMMAND_START
        goal.mission_id = mission_id
        goal.target_pose = self._make_pose(x_value, y_value, yaw_deg)
        goal.constraints_json = constraints_json
        goal.priority = int(priority)
        goal.timeout_sec = float(timeout_sec)
        self._request_action_goal(
            label='手动切行走',
            client=self.walk_cli,
            goal=goal,
            stop_dispatcher_first=True,
        )

    def request_manual_walk_cancel(
        self,
        *,
        mission_id: str,
        timeout_sec: float = 5.0,
    ) -> None:
        goal = WalkMission.Goal()
        goal.command = COMMAND_CANCEL
        goal.mission_id = mission_id
        goal.target_pose.header.frame_id = 'map'
        goal.constraints_json = '{}'
        goal.priority = 1
        goal.timeout_sec = float(timeout_sec)
        self._request_action_goal(
            label='停止行走',
            client=self.walk_cli,
            goal=goal,
            stop_dispatcher_first=False,
        )

    def request_manual_dig_start(
        self,
        *,
        mission_id: str,
        target_zone: str,
        process_parameters_json: str,
        safety_boundary_json: str,
        priority: int,
        timeout_sec: float = 90.0,
    ) -> None:
        goal = DigMission.Goal()
        goal.command = COMMAND_START
        goal.mission_id = mission_id
        goal.target_zone = target_zone
        goal.process_parameters_json = process_parameters_json
        goal.safety_boundary_json = safety_boundary_json
        goal.priority = int(priority)
        goal.timeout_sec = float(timeout_sec)
        self._request_action_goal(
            label='手动切挖掘',
            client=self.dig_cli,
            goal=goal,
            stop_dispatcher_first=True,
        )

    def request_manual_dig_cancel(
        self,
        *,
        mission_id: str,
        target_zone: str,
        timeout_sec: float = 10.0,
    ) -> None:
        goal = DigMission.Goal()
        goal.command = COMMAND_CANCEL
        goal.mission_id = mission_id
        goal.target_zone = target_zone
        goal.process_parameters_json = '{}'
        goal.safety_boundary_json = '{}'
        goal.priority = 1
        goal.timeout_sec = float(timeout_sec)
        self._request_action_goal(
            label='停止挖掘',
            client=self.dig_cli,
            goal=goal,
            stop_dispatcher_first=False,
        )

    def submit_demo_mission(self) -> None:
        self.request_submit_mission(
            {
                'mission_id': f'demo-{int(time.time())}',
                'use_material_target': True,
                'current_pose_x': 0.0,
                'current_pose_y': 0.0,
                'walk_target_x': 0.0,
                'walk_target_y': 0.0,
                'walk_target_yaw_deg': 0.0,
                'material_reference_x': 10.0,
                'material_reference_y': 0.0,
                'desired_standoff_m': 3.0,
                'outline_points': [(8.0, 0.0), (10.0, 0.0), (12.0, 0.0)],
                'material_profile_json': '{"material_kind":"ore","target_strategy":"auto"}',
                'target_planner_constraints_json': '{"edge_near_threshold_m":1.5}',
                'dig_target_zone': 'bench-A',
                'walk_constraints_json': '{"simulate":"success","duration_sec":3.0}',
                'dig_parameters_json': '{"simulate":"success","duration_sec":4.0,"material_volume":8.5}',
                'priority': 1,
            }
        )

    def _request_action_goal(
        self,
        *,
        label: str,
        client: Any,
        goal: Any,
        stop_dispatcher_first: bool,
    ) -> None:
        def worker() -> None:
            if stop_dispatcher_first:
                if not self.stop_cli.wait_for_service(timeout_sec=2.0):
                    self._append_event('HMI', f'{label}失败：dispatcher stop service 不可用')
                    return
                stop_future = self.stop_cli.call_async(Trigger.Request())
                deadline = time.monotonic() + 3.0
                while not stop_future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not stop_future.done():
                    self._append_event('HMI', f'{label}失败：停止自动模式超时')
                    return
                try:
                    stop_response = stop_future.result()
                except Exception as exc:  # pragma: no cover
                    self._append_event('HMI', f'{label}失败：停止自动模式异常 {exc}')
                    return
                if not stop_response.success:
                    self._append_event('HMI', f'{label}失败：停止自动模式失败 {stop_response.message}')
                    return
                self._append_event('HMI', '已接管自动调度，进入手动子系统控制')

            if not client.wait_for_server(timeout_sec=3.0):
                self._append_event('HMI', f'{label}失败：action server 不可用')
                return
            send_future = client.send_goal_async(goal)
            send_future.add_done_callback(lambda fut: self._on_action_goal_sent(label, fut))

        threading.Thread(target=worker, daemon=True).start()

    def _on_action_goal_sent(self, label: str, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._append_event('HMI', f'{label}失败：{exc}')
            return
        if not goal_handle.accepted:
            self._append_event('HMI', f'{label}失败：goal 被拒绝')
            return
        self._append_event('HMI', f'{label}已发送，等待子系统确认')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_action_result(label, fut))

    def _on_action_result(self, label: str, future: Any) -> None:
        try:
            result_wrapper = future.result()
        except Exception as exc:  # pragma: no cover
            self._append_event('HMI', f'{label}失败：result 异常 {exc}')
            return
        result = result_wrapper.result
        accepted = getattr(result, 'accepted', True)
        message = getattr(result, 'message', '')
        state = '成功' if accepted else '失败'
        suffix = f'：{message}' if message else ''
        self._append_event('HMI', f'{label}{state}{suffix}')

    def _build_submit_request(self, payload: dict[str, Any]) -> SubmitMission.Request:
        request = SubmitMission.Request()
        request.mission_id = payload['mission_id']
        request.use_material_target = bool(payload['use_material_target'])
        request.current_pose = self._make_pose(payload['current_pose_x'], payload['current_pose_y'], 0.0)
        request.walk_target = self._make_pose(
            payload['walk_target_x'],
            payload['walk_target_y'],
            payload['walk_target_yaw_deg'],
        )
        request.material_reference_pose = self._make_pose(
            payload['material_reference_x'],
            payload['material_reference_y'],
            0.0,
        )
        request.material_outline = [Point(x=x_value, y=y_value, z=0.0) for x_value, y_value in payload['outline_points']]
        request.desired_standoff_m = float(payload['desired_standoff_m'])
        request.material_profile_json = payload['material_profile_json']
        request.target_planner_constraints_json = payload['target_planner_constraints_json']
        request.dig_target_zone = payload['dig_target_zone']
        request.walk_constraints_json = payload['walk_constraints_json']
        request.dig_parameters_json = payload['dig_parameters_json']
        request.priority = int(payload['priority'])
        return request

    def _on_submit_done(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self._append_event('HMI', f'提交任务失败：{exc}')
            return
        if not response.accepted:
            self._append_event('HMI', f'任务被拒绝：{response.message}')
            return
        self._append_event(
            'HMI',
            '任务已接收：'
            f"{response.message} resolved=({response.resolved_walk_target.pose.position.x:.2f},"
            f" {response.resolved_walk_target.pose.position.y:.2f})",
        )

    @staticmethod
    def _make_pose(x_value: float, y_value: float, yaw_deg: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x_value)
        msg.pose.position.y = float(y_value)
        yaw_rad = math.radians(float(yaw_deg))
        msg.pose.orientation.z = math.sin(yaw_rad * 0.5)
        msg.pose.orientation.w = math.cos(yaw_rad * 0.5)
        return msg


class IntegratedOperatorHmi:
    def __init__(self, root: tk.Tk, backend: MissionOperatorBackend) -> None:
        self.root = root
        self.backend = backend
        self._last_event_signature: tuple[str, ...] = ()
        self._mobility_view_transform: ViewTransform | None = None
        self._goal_pick_anchor_world: tuple[float, float] | None = None
        self._goal_pick_current_world: tuple[float, float] | None = None

        self.root.title('Integrated Mission Planner HMI')
        self.root.geometry('1680x980')
        self.root.configure(bg='#e7dfd2')
        self._build_style()
        self._build_layout()
        self._fill_defaults()
        self._refresh_ui()

    def _build_style(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('Root.TFrame', background='#e7dfd2')
        style.configure('Card.TFrame', background='#f8f4ec', relief='flat')
        style.configure('Title.TLabel', background='#e7dfd2', foreground='#2f2419', font=('DejaVu Sans', 18, 'bold'))
        style.configure('CardTitle.TLabel', background='#f8f4ec', foreground='#6a4b2d', font=('DejaVu Sans', 11, 'bold'))
        style.configure('CardValue.TLabel', background='#f8f4ec', foreground='#1f2933', font=('DejaVu Sans', 15, 'bold'))
        style.configure('Section.TLabel', background='#f8f4ec', foreground='#2f2419', font=('DejaVu Sans', 12, 'bold'))
        style.configure('TNotebook', background='#e7dfd2', borderwidth=0)
        style.configure('TNotebook.Tab', padding=(16, 8), font=('DejaVu Sans', 11, 'bold'))
        style.map('TNotebook.Tab', background=[('selected', '#2f2419')], foreground=[('selected', '#f8f4ec')])
        style.configure('Accent.TButton', font=('DejaVu Sans', 10, 'bold'))

    def _build_layout(self) -> None:
        container = ttk.Frame(self.root, style='Root.TFrame', padding=12)
        container.pack(fill='both', expand=True)

        header = ttk.Frame(container, style='Root.TFrame')
        header.pack(fill='x')
        ttk.Label(header, text='Integrated Mission Planner HMI', style='Title.TLabel').pack(anchor='w')

        summary = ttk.Frame(container, style='Root.TFrame')
        summary.pack(fill='x', pady=(10, 12))
        self.summary_vars = {
            'mode': tk.StringVar(value='IDLE'),
            'mission': tk.StringVar(value='--'),
            'walk': tk.StringVar(value='idle'),
            'dig': tk.StringVar(value='idle'),
            'plc': tk.StringVar(value='not_ready'),
        }
        cards = [
            ('大规划模式', 'mode'),
            ('任务编号', 'mission'),
            ('行走状态', 'walk'),
            ('挖掘状态', 'dig'),
            ('PLC', 'plc'),
        ]
        for title, key in cards:
            card = ttk.Frame(summary, style='Card.TFrame', padding=10)
            card.pack(side='left', fill='x', expand=True, padx=(0, 8))
            ttk.Label(card, text=title, style='CardTitle.TLabel').pack(anchor='w')
            ttk.Label(card, textvariable=self.summary_vars[key], style='CardValue.TLabel').pack(anchor='w', pady=(6, 0))

        body = ttk.Frame(container, style='Root.TFrame')
        body.pack(fill='both', expand=True)

        left = ttk.Frame(body, style='Root.TFrame')
        left.pack(side='left', fill='both', expand=True)
        right = ttk.Frame(body, style='Root.TFrame', width=420)
        right.pack(side='left', fill='both', padx=(12, 0))
        right.pack_propagate(False)

        self.notebook = ttk.Notebook(left)
        self.notebook.pack(fill='both', expand=True)
        self.mission_tab = ttk.Frame(self.notebook, style='Card.TFrame', padding=12)
        self.mobility_tab = ttk.Frame(self.notebook, style='Card.TFrame', padding=12)
        self.excavation_tab = ttk.Frame(self.notebook, style='Card.TFrame', padding=12)
        self.notebook.add(self.mission_tab, text='大规划')
        self.notebook.add(self.mobility_tab, text='行走规划')
        self.notebook.add(self.excavation_tab, text='挖掘规划')

        self._build_mission_tab()
        self._build_mobility_tab()
        self._build_excavation_tab()

        self.event_text = scrolledtext.ScrolledText(
            right,
            wrap='word',
            height=20,
            bg='#161b22',
            fg='#e6edf3',
            insertbackground='#e6edf3',
            font=('DejaVu Sans Mono', 10),
            relief='flat',
            padx=10,
            pady=10,
        )
        event_header = ttk.Frame(right, style='Card.TFrame', padding=12)
        event_header.pack(fill='x')
        ttk.Label(event_header, text='系统事件流', style='Section.TLabel').pack(anchor='w')
        self.event_text.pack(fill='both', expand=True)

    def _build_mission_tab(self) -> None:
        controls = ttk.Frame(self.mission_tab, style='Card.TFrame')
        controls.pack(fill='x')
        ttk.Button(controls, text='开始自动', command=self.backend.request_start, style='Accent.TButton').pack(side='left', padx=(0, 8))
        ttk.Button(controls, text='停止/取消', command=self.backend.request_stop, style='Accent.TButton').pack(side='left', padx=(0, 8))
        ttk.Button(controls, text='恢复', command=self.backend.request_recover, style='Accent.TButton').pack(side='left', padx=(0, 8))
        ttk.Button(controls, text='提交示例任务', command=self.backend.submit_demo_mission, style='Accent.TButton').pack(side='left')

        quick_nav = ttk.Frame(self.mission_tab, style='Card.TFrame')
        quick_nav.pack(fill='x', pady=(10, 0))
        ttk.Label(quick_nav, text='演示快捷切换', style='Section.TLabel').pack(side='left', padx=(0, 10))
        ttk.Button(quick_nav, text='查看行走页', command=lambda: self.notebook.select(self.mobility_tab)).pack(side='left', padx=(0, 8))
        ttk.Button(quick_nav, text='查看挖掘页', command=lambda: self.notebook.select(self.excavation_tab)).pack(side='left', padx=(0, 8))
        ttk.Button(quick_nav, text='查看大规划页', command=lambda: self.notebook.select(self.mission_tab)).pack(side='left')

        form = ttk.Frame(self.mission_tab, style='Card.TFrame')
        form.pack(fill='both', expand=True, pady=(14, 0))
        form.columnconfigure(1, weight=1)
        form.columnconfigure(3, weight=1)

        self.mission_id_var = tk.StringVar()
        self.use_material_target_var = tk.BooleanVar(value=True)
        self.current_pose_x_var = tk.DoubleVar(value=0.0)
        self.current_pose_y_var = tk.DoubleVar(value=0.0)
        self.walk_target_x_var = tk.DoubleVar(value=5.0)
        self.walk_target_y_var = tk.DoubleVar(value=0.0)
        self.walk_target_yaw_var = tk.DoubleVar(value=0.0)
        self.material_reference_x_var = tk.DoubleVar(value=10.0)
        self.material_reference_y_var = tk.DoubleVar(value=0.0)
        self.desired_standoff_var = tk.DoubleVar(value=3.0)
        self.dig_target_zone_var = tk.StringVar(value='bench-A')
        self.priority_var = tk.IntVar(value=1)

        row = 0
        self._add_entry(form, row, '任务 ID', self.mission_id_var)
        self._add_check(form, row, '使用停靠点计算', self.use_material_target_var, column=2)
        row += 1
        self._add_spin(form, row, '当前位置 X', self.current_pose_x_var)
        self._add_spin(form, row, '当前位置 Y', self.current_pose_y_var, column=2)
        row += 1
        self._add_spin(form, row, '行走目标 X', self.walk_target_x_var)
        self._add_spin(form, row, '行走目标 Y', self.walk_target_y_var, column=2)
        row += 1
        self._add_spin(form, row, '行走目标 Yaw', self.walk_target_yaw_var)
        self._add_spin(form, row, '优先级', self.priority_var, column=2)
        row += 1
        self._add_spin(form, row, '物料参考 X', self.material_reference_x_var)
        self._add_spin(form, row, '物料参考 Y', self.material_reference_y_var, column=2)
        row += 1
        self._add_spin(form, row, '停靠距离', self.desired_standoff_var)
        self._add_entry(form, row, '挖掘区域', self.dig_target_zone_var, column=2)
        row += 1

        ttk.Label(form, text='物料轮廓点 (每行 x,y)', style='Section.TLabel').grid(row=row, column=0, sticky='w', pady=(10, 4))
        ttk.Label(form, text='物料/目标约束 JSON', style='Section.TLabel').grid(row=row, column=2, sticky='w', pady=(10, 4))
        row += 1
        self.outline_text = scrolledtext.ScrolledText(form, height=8, width=34, font=('DejaVu Sans Mono', 10))
        self.outline_text.grid(row=row, column=0, columnspan=2, sticky='nsew', padx=(0, 10))
        right_json_frame = ttk.Frame(form, style='Card.TFrame')
        right_json_frame.grid(row=row, column=2, columnspan=2, sticky='nsew')
        form.rowconfigure(row, weight=1)

        ttk.Label(right_json_frame, text='material_profile_json', style='Section.TLabel').pack(anchor='w')
        self.material_profile_text = scrolledtext.ScrolledText(right_json_frame, height=4, font=('DejaVu Sans Mono', 10))
        self.material_profile_text.pack(fill='x', pady=(4, 10))
        ttk.Label(right_json_frame, text='target_planner_constraints_json', style='Section.TLabel').pack(anchor='w')
        self.target_constraints_text = scrolledtext.ScrolledText(right_json_frame, height=4, font=('DejaVu Sans Mono', 10))
        self.target_constraints_text.pack(fill='x', pady=(4, 10))
        ttk.Label(right_json_frame, text='walk_constraints_json / dig_parameters_json', style='Section.TLabel').pack(anchor='w')
        self.walk_constraints_text = scrolledtext.ScrolledText(right_json_frame, height=3, font=('DejaVu Sans Mono', 10))
        self.walk_constraints_text.pack(fill='x', pady=(4, 6))
        self.dig_parameters_text = scrolledtext.ScrolledText(right_json_frame, height=3, font=('DejaVu Sans Mono', 10))
        self.dig_parameters_text.pack(fill='x')

        row += 1
        submit_row = ttk.Frame(form, style='Card.TFrame')
        submit_row.grid(row=row, column=0, columnspan=4, sticky='w', pady=(12, 0))
        ttk.Button(submit_row, text='提交当前表单任务', command=self._submit_form_mission, style='Accent.TButton').pack(side='left')
        ttk.Button(submit_row, text='手动切行走', command=self._manual_switch_walk, style='Accent.TButton').pack(side='left', padx=(8, 0))
        ttk.Button(submit_row, text='停止行走', command=self._manual_cancel_walk).pack(side='left', padx=(8, 0))
        ttk.Button(submit_row, text='手动切挖掘', command=self._manual_switch_dig, style='Accent.TButton').pack(side='left', padx=(8, 0))
        ttk.Button(submit_row, text='停止挖掘', command=self._manual_cancel_dig).pack(side='left', padx=(8, 0))

    def _build_mobility_tab(self) -> None:
        container = ttk.Frame(self.mobility_tab, style='Card.TFrame')
        container.pack(fill='both', expand=True)
        container.columnconfigure(0, weight=5)
        container.columnconfigure(1, weight=2)
        container.rowconfigure(0, weight=1)
        container.rowconfigure(1, weight=1)

        self.mobility_canvas = tk.Canvas(container, bg='#111827', highlightthickness=0)
        self.mobility_canvas.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=(0, 12))
        self.mobility_canvas.bind('<ButtonPress-1>', self._on_mobility_canvas_press)
        self.mobility_canvas.bind('<B1-Motion>', self._on_mobility_canvas_drag)
        self.mobility_canvas.bind('<ButtonRelease-1>', self._on_mobility_canvas_release)

        side_panel = ttk.Frame(container, style='Card.TFrame')
        side_panel.grid(row=0, column=1, sticky='nsew')
        side_panel.columnconfigure(0, weight=1)

        self.mobility_info_vars = {
            'pose': tk.StringVar(value='--'),
            'speed': tk.StringVar(value='--'),
            'source': tk.StringVar(value='--'),
            'goal': tk.StringVar(value='--'),
            'auto_goal': tk.StringVar(value='--'),
            'boundary': tk.StringVar(value='0 pts'),
            'path': tk.StringVar(value='0 poses'),
            'cmd_vel': tk.StringVar(value='linear=0.00, angular=0.00'),
            'track': tk.StringVar(value='L=0.00, R=0.00'),
        }
        info_grid = ttk.Frame(side_panel, style='Card.TFrame')
        info_grid.grid(row=0, column=0, sticky='ew')
        labels = [
            ('当前位置', 'pose'),
            ('速度', 'speed'),
            ('定位源', 'source'),
            ('手动目标', 'goal'),
            ('自动目标', 'auto_goal'),
            ('物料边界', 'boundary'),
            ('规划路径', 'path'),
            ('cmd_vel', 'cmd_vel'),
            ('履带速度', 'track'),
        ]
        for index, (title, key) in enumerate(labels):
            frame = ttk.Frame(info_grid, style='Card.TFrame', padding=6)
            frame.grid(row=index // 2, column=index % 2, sticky='nsew', padx=4, pady=4)
            info_grid.columnconfigure(index % 2, weight=1)
            ttk.Label(frame, text=title, style='CardTitle.TLabel').pack(anchor='w')
            ttk.Label(frame, textvariable=self.mobility_info_vars[key], style='CardValue.TLabel').pack(anchor='w')

        controls = ttk.LabelFrame(side_panel, text='显示与目标', padding=10)
        controls.grid(row=1, column=0, sticky='ew', pady=(12, 0))
        self.show_map_var = tk.BooleanVar(value=True)
        self.show_gcm_var = tk.BooleanVar(value=True)
        self.show_lcm_var = tk.BooleanVar(value=True)
        self.show_robot_var = tk.BooleanVar(value=True)
        self.show_plan_var = tk.BooleanVar(value=True)
        self.show_goal_var = tk.BooleanVar(value=True)
        self.show_auto_goal_var = tk.BooleanVar(value=True)
        self.show_boundary_var = tk.BooleanVar(value=True)
        self.pick_goal_mode_var = tk.BooleanVar(value=False)
        for row, (label, variable) in enumerate(
            (
                ('显示 map', self.show_map_var),
                ('显示 global_costmap', self.show_gcm_var),
                ('显示 local_costmap', self.show_lcm_var),
                ('显示机器人', self.show_robot_var),
                ('显示规划路径', self.show_plan_var),
                ('显示手动目标', self.show_goal_var),
                ('显示自动目标', self.show_auto_goal_var),
                ('显示物料边界', self.show_boundary_var),
            )
        ):
            ttk.Checkbutton(controls, text=label, variable=variable).grid(row=row // 2, column=row % 2, sticky='w', padx=(0, 12), pady=2)
        ttk.Checkbutton(controls, text='点选目标模式', variable=self.pick_goal_mode_var).grid(
            row=4, column=0, columnspan=2, sticky='w', pady=(4, 0)
        )

        goal_frame = ttk.LabelFrame(side_panel, text='手动目标下发', padding=10)
        goal_frame.grid(row=2, column=0, sticky='ew', pady=(12, 0))
        goal_frame.columnconfigure(1, weight=1)
        self.goal_x_var = tk.DoubleVar(value=5.0)
        self.goal_y_var = tk.DoubleVar(value=0.0)
        self.goal_yaw_var = tk.DoubleVar(value=0.0)
        self._add_spin(goal_frame, 0, 'Goal X', self.goal_x_var)
        self._add_spin(goal_frame, 1, 'Goal Y', self.goal_y_var)
        self._add_spin(goal_frame, 2, 'Goal Yaw', self.goal_yaw_var)
        ttk.Button(goal_frame, text='发送目标点', command=self._publish_manual_goal, style='Accent.TButton').grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=(8, 0)
        )
        ttk.Label(
            goal_frame,
            text='提示: 开启点选目标模式后, 地图中按下设置位置, 拖动设置朝向, 松开即发布。',
            style='CardTitle.TLabel',
            wraplength=260,
        ).grid(row=4, column=0, columnspan=2, sticky='w', pady=(8, 0))

        indicator_frame = ttk.LabelFrame(side_panel, text='状态灯', padding=10)
        indicator_frame.grid(row=3, column=0, sticky='ew', pady=(12, 0))
        self.mobility_indicator_labels: dict[str, tk.Label] = {}
        for row, key in enumerate(('map', 'planner', 'controller', 'plc', 'localization')):
            cell = ttk.Frame(indicator_frame, style='Card.TFrame')
            cell.grid(row=row, column=0, sticky='ew', pady=2)
            ttk.Label(cell, text=key, style='CardTitle.TLabel').pack(side='left')
            led = tk.Label(cell, width=2, bg='#7f1d1d', relief='flat')
            led.pack(side='right', padx=(10, 0))
            self.mobility_indicator_labels[key] = led

        self.mobility_track_canvas = tk.Canvas(container, bg='#fdfaf4', highlightthickness=0)
        self.mobility_track_canvas.grid(row=1, column=1, sticky='nsew', pady=(12, 0))

    def _build_excavation_tab(self) -> None:
        top = ttk.Frame(self.excavation_tab, style='Card.TFrame')
        top.pack(fill='x')
        self.excavation_info_vars = {
            'phase': tk.StringVar(value='idle'),
            'detail': tk.StringVar(value='--'),
            'swing': tk.StringVar(value='0.0 deg'),
            'files': tk.StringVar(value='等待实时轨迹 topic'),
            'optimizer': tk.StringVar(value='eval=0 obj=-- fill=--'),
        }
        for title, key in (
            ('挖掘阶段', 'phase'),
            ('阶段详情', 'detail'),
            ('回转角度', 'swing'),
            ('实时轨迹流', 'files'),
            ('优化中间态', 'optimizer'),
        ):
            frame = ttk.Frame(top, style='Card.TFrame', padding=8)
            frame.pack(fill='x', pady=4)
            ttk.Label(frame, text=title, style='CardTitle.TLabel').pack(anchor='w')
            ttk.Label(frame, textvariable=self.excavation_info_vars[key], style='CardValue.TLabel').pack(anchor='w')
        canvas_frame = ttk.Frame(self.excavation_tab, style='Card.TFrame')
        canvas_frame.pack(fill='both', expand=True, pady=(12, 0))
        canvas_frame.rowconfigure(0, weight=3)
        canvas_frame.rowconfigure(1, weight=2)
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.columnconfigure(1, weight=1)
        self.excavation_xy_canvas = tk.Canvas(canvas_frame, bg='#fff8ef', highlightthickness=0)
        self.excavation_xy_canvas.grid(row=0, column=0, columnspan=2, sticky='nsew')
        self.excavation_speed_canvas = tk.Canvas(canvas_frame, bg='#fff8ef', highlightthickness=0)
        self.excavation_speed_canvas.grid(row=1, column=0, sticky='nsew', pady=(8, 0), padx=(0, 6))
        self.excavation_rotation_canvas = tk.Canvas(canvas_frame, bg='#fff8ef', highlightthickness=0)
        self.excavation_rotation_canvas.grid(row=1, column=1, sticky='nsew', pady=(8, 0), padx=(6, 0))

    def _fill_defaults(self) -> None:
        self.mission_id_var.set(f'hmi-{int(time.time())}')
        self.outline_text.insert('1.0', '8.0,0.0\n10.0,0.0\n12.0,0.0\n')
        self.material_profile_text.insert('1.0', '{\n  "material_kind": "ore",\n  "target_strategy": "auto"\n}')
        self.target_constraints_text.insert('1.0', '{\n  "edge_near_threshold_m": 1.5\n}')
        self.walk_constraints_text.insert('1.0', '{\n  "simulate": "success",\n  "duration_sec": 3.0\n}')
        self.dig_parameters_text.insert('1.0', '{\n  "simulate": "success",\n  "duration_sec": 4.0,\n  "material_volume": 8.5\n}')

    def _add_entry(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, *, column: int = 0) -> None:
        ttk.Label(parent, text=label, style='Section.TLabel').grid(row=row, column=column, sticky='w', pady=4)
        ttk.Entry(parent, textvariable=variable).grid(row=row, column=column + 1, sticky='ew', padx=(8, 12), pady=4)

    def _add_spin(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, *, column: int = 0) -> None:
        ttk.Label(parent, text=label, style='Section.TLabel').grid(row=row, column=column, sticky='w', pady=4)
        ttk.Spinbox(parent, textvariable=variable, from_=-10000.0, to=10000.0, increment=0.1).grid(
            row=row, column=column + 1, sticky='ew', padx=(8, 12), pady=4
        )

    def _add_check(self, parent: ttk.Frame, row: int, label: str, variable: tk.BooleanVar, *, column: int = 0) -> None:
        ttk.Checkbutton(parent, text=label, variable=variable).grid(row=row, column=column, sticky='w', padx=(8, 12), pady=4)

    def _submit_form_mission(self) -> None:
        try:
            payload = {
                'mission_id': self.mission_id_var.get().strip() or f'hmi-{int(time.time())}',
                'use_material_target': self.use_material_target_var.get(),
                'current_pose_x': float(self.current_pose_x_var.get()),
                'current_pose_y': float(self.current_pose_y_var.get()),
                'walk_target_x': float(self.walk_target_x_var.get()),
                'walk_target_y': float(self.walk_target_y_var.get()),
                'walk_target_yaw_deg': float(self.walk_target_yaw_var.get()),
                'material_reference_x': float(self.material_reference_x_var.get()),
                'material_reference_y': float(self.material_reference_y_var.get()),
                'desired_standoff_m': float(self.desired_standoff_var.get()),
                'outline_points': parse_outline_text(self.outline_text.get('1.0', 'end')),
                'material_profile_json': normalize_json_text(self.material_profile_text.get('1.0', 'end')),
                'target_planner_constraints_json': normalize_json_text(self.target_constraints_text.get('1.0', 'end')),
                'dig_target_zone': self.dig_target_zone_var.get().strip() or 'bench-A',
                'walk_constraints_json': normalize_json_text(self.walk_constraints_text.get('1.0', 'end')),
                'dig_parameters_json': normalize_json_text(self.dig_parameters_text.get('1.0', 'end')),
                'priority': int(self.priority_var.get()),
            }
        except Exception as exc:
            messagebox.showerror('提交任务失败', str(exc))
            return
        self.backend.request_submit_mission(payload)

    def _publish_manual_goal(self) -> None:
        self.backend.publish_goal(
            float(self.goal_x_var.get()),
            float(self.goal_y_var.get()),
            float(self.goal_yaw_var.get()),
        )

    def _manual_switch_walk(self) -> None:
        mission_id = self.mission_id_var.get().strip() or f'manual-walk-{int(time.time())}'
        self.notebook.select(self.mobility_tab)
        self.backend.request_manual_walk_start(
            x_value=float(self.goal_x_var.get()),
            y_value=float(self.goal_y_var.get()),
            yaw_deg=float(self.goal_yaw_var.get()),
            constraints_json=normalize_json_text(self.walk_constraints_text.get('1.0', 'end')),
            mission_id=mission_id,
            priority=int(self.priority_var.get()),
        )

    def _manual_cancel_walk(self) -> None:
        mission_id = self.mission_id_var.get().strip() or 'manual-walk'
        self.backend.request_manual_walk_cancel(mission_id=mission_id)

    def _manual_switch_dig(self) -> None:
        mission_id = self.mission_id_var.get().strip() or f'manual-dig-{int(time.time())}'
        self.notebook.select(self.excavation_tab)
        self.backend.request_manual_dig_start(
            mission_id=mission_id,
            target_zone=self.dig_target_zone_var.get().strip() or 'bench-A',
            process_parameters_json=normalize_json_text(self.dig_parameters_text.get('1.0', 'end')),
            safety_boundary_json='{}',
            priority=int(self.priority_var.get()),
        )

    def _manual_cancel_dig(self) -> None:
        mission_id = self.mission_id_var.get().strip() or 'manual-dig'
        self.backend.request_manual_dig_cancel(
            mission_id=mission_id,
            target_zone=self.dig_target_zone_var.get().strip() or 'bench-A',
        )

    def _on_mobility_canvas_press(self, event: tk.Event) -> None:
        if not self.pick_goal_mode_var.get() or self._mobility_view_transform is None:
            return
        world_point = screen_to_world(float(event.x), float(event.y), self._mobility_view_transform)
        self._goal_pick_anchor_world = world_point
        self._goal_pick_current_world = world_point

    def _on_mobility_canvas_drag(self, event: tk.Event) -> None:
        if self._goal_pick_anchor_world is None or self._mobility_view_transform is None:
            return
        self._goal_pick_current_world = screen_to_world(float(event.x), float(event.y), self._mobility_view_transform)

    def _on_mobility_canvas_release(self, event: tk.Event) -> None:
        if self._goal_pick_anchor_world is None or self._mobility_view_transform is None:
            return
        release_world = screen_to_world(float(event.x), float(event.y), self._mobility_view_transform)
        anchor = self._goal_pick_anchor_world
        self._goal_pick_current_world = release_world
        dx = release_world[0] - anchor[0]
        dy = release_world[1] - anchor[1]
        if math.hypot(dx, dy) > 0.15:
            yaw_deg = math.degrees(math.atan2(dy, dx))
        else:
            yaw_deg = float(self.goal_yaw_var.get())
        self.goal_x_var.set(anchor[0])
        self.goal_y_var.set(anchor[1])
        self.goal_yaw_var.set(yaw_deg)
        self.backend.publish_goal(anchor[0], anchor[1], yaw_deg)
        self._goal_pick_anchor_world = None
        self._goal_pick_current_world = None

    def _refresh_ui(self) -> None:
        snapshot = self.backend.snapshot()
        self._refresh_summary(snapshot)
        self._refresh_events(snapshot)
        self._refresh_mobility(snapshot)
        self._refresh_excavation(snapshot)
        self.root.after(250, self._refresh_ui)

    def _refresh_summary(self, snapshot: dict[str, Any]) -> None:
        self.summary_vars['mode'].set(snapshot['mode']['name'])
        self.summary_vars['mission'].set(snapshot['mode']['mission_id'] or '--')
        self.summary_vars['walk'].set(snapshot['walk_status']['phase'])
        self.summary_vars['dig'].set(snapshot['dig_status']['phase'])
        plc = snapshot['plc_status']
        if plc['fault_active']:
            plc_text = f"FAULT({plc['fault_code']})"
        elif plc['machine_ready'] and plc['safe_to_walk'] and plc['safe_to_dig']:
            plc_text = f"ready/{plc['source']}"
        else:
            plc_text = f"blocked/{plc['source']}"
        self.summary_vars['plc'].set(plc_text)

    def _refresh_events(self, snapshot: dict[str, Any]) -> None:
        events = tuple(snapshot['events'])
        if events == self._last_event_signature:
            return
        self._last_event_signature = events
        self.event_text.configure(state='normal')
        self.event_text.delete('1.0', 'end')
        self.event_text.insert('1.0', '\n'.join(reversed(events)))
        self.event_text.configure(state='disabled')

    def _refresh_mobility(self, snapshot: dict[str, Any]) -> None:
        current_pose = snapshot['current_pose']
        self.mobility_info_vars['pose'].set(
            '--' if not current_pose else f"x={current_pose['x']:.2f}, y={current_pose['y']:.2f}, yaw={current_pose['yaw_deg']:.1f}"
        )
        self.mobility_info_vars['speed'].set('--' if not current_pose else f"{current_pose['speed_mps']:.2f} m/s")
        self.mobility_info_vars['source'].set(snapshot['pose_source'])
        self.mobility_info_vars['goal'].set(self._format_pose(snapshot['goal_pose']))
        self.mobility_info_vars['auto_goal'].set(self._format_pose(snapshot['auto_goal_pose']))
        self.mobility_info_vars['path'].set(f"{len(snapshot['plan_points'])} poses")
        self.mobility_info_vars['boundary'].set(f"{len(snapshot['material_boundary'])} pts")
        cmd_vel = snapshot['cmd_vel']
        self.mobility_info_vars['cmd_vel'].set(f"linear={cmd_vel['linear']:.2f}, angular={cmd_vel['angular']:.2f}")
        self.mobility_info_vars['track'].set(
            f"L={cmd_vel['left_track']:.2f} m/s, R={cmd_vel['right_track']:.2f} m/s"
        )
        self._refresh_mobility_indicators(snapshot)
        self._draw_mobility_canvas(snapshot)
        self._draw_mobility_track_canvas(snapshot)

    def _refresh_mobility_indicators(self, snapshot: dict[str, Any]) -> None:
        states = {
            'map': bool(snapshot['map_points']),
            'planner': bool(snapshot['plan_points']) or snapshot['walk_status']['active'],
            'controller': abs(snapshot['cmd_vel']['linear']) > 1e-3 or abs(snapshot['cmd_vel']['angular']) > 1e-3,
            'plc': snapshot['plc_status']['machine_ready'] and not snapshot['plc_status']['fault_active'],
            'localization': snapshot['current_pose'] is not None,
        }
        for key, active in states.items():
            self.mobility_indicator_labels[key].configure(bg='#15803d' if active else '#7f1d1d')

    def _draw_mobility_canvas(self, snapshot: dict[str, Any]) -> None:
        canvas = self.mobility_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 480)
        map_points = downsample_points(snapshot['map_points'], max_points=2200) if self.show_map_var.get() else []
        global_costmap = downsample_points(snapshot['global_costmap_points'], max_points=1200) if self.show_gcm_var.get() else []
        local_costmap = downsample_points(snapshot['local_costmap_points'], max_points=1200) if self.show_lcm_var.get() else []
        boundary = downsample_points(snapshot['material_boundary'], max_points=200)
        path = downsample_points(snapshot['plan_points'], max_points=400)
        current = []
        goal = []
        auto_goal = []
        if snapshot['current_pose']:
            current = [(snapshot['current_pose']['x'], snapshot['current_pose']['y'])]
        if snapshot['goal_pose']:
            goal = [(snapshot['goal_pose'][0], snapshot['goal_pose'][1])]
        if snapshot['auto_goal_pose']:
            auto_goal = [(snapshot['auto_goal_pose'][0], snapshot['auto_goal_pose'][1])]
        bounds = compute_bounds([map_points, global_costmap, local_costmap, boundary, path, current, goal, auto_goal])
        transform = compute_view_transform(bounds, width, height)
        self._mobility_view_transform = transform
        for color, points, width_px in (
            ('#d48b31', boundary if self.show_boundary_var.get() else [], 3),
            ('#2f6eb5', path if self.show_plan_var.get() else [], 3),
        ):
            if len(points) >= 2:
                projected = project_points_with_transform(points, transform)
                canvas.create_line(*[value for point in projected for value in point], fill=color, width=width_px, smooth=True)
        for points, fill_color, size_px in (
            (map_points, '#6b7280', 1),
            (global_costmap, '#ef4444', 2),
            (local_costmap, '#38bdf8', 2),
        ):
            for x_value, y_value in project_points_with_transform(points, transform):
                canvas.create_rectangle(
                    x_value - size_px,
                    y_value - size_px,
                    x_value + size_px,
                    y_value + size_px,
                    fill=fill_color,
                    outline='',
                )
        for points, fill_color, outline_color, radius in (
            (goal if self.show_goal_var.get() else [], '#3f9c35', '#245b1e', 5),
            (auto_goal if self.show_auto_goal_var.get() else [], '#d97706', '#a16207', 5),
        ):
            for x_value, y_value in project_points_with_transform(points, transform):
                canvas.create_oval(x_value - radius, y_value - radius, x_value + radius, y_value + radius, fill=fill_color, outline=outline_color)
        current_pose = snapshot['current_pose']
        if self.show_robot_var.get() and current_pose:
            projected_current = project_points_with_transform(current, transform)
            if projected_current:
                x_value, y_value = projected_current[0]
                yaw = math.radians(float(current_pose['yaw_deg']))
                heading = (x_value + math.cos(yaw) * 14.0, y_value - math.sin(yaw) * 14.0)
                canvas.create_oval(x_value - 6, y_value - 6, x_value + 6, y_value + 6, fill='#f8fafc', outline='#0f172a')
                canvas.create_line(x_value, y_value, heading[0], heading[1], fill='#f8fafc', width=3)
        if self.pick_goal_mode_var.get() and self._goal_pick_anchor_world is not None:
            anchor_xy = project_points_with_transform([self._goal_pick_anchor_world], transform)[0]
            canvas.create_oval(
                anchor_xy[0] - 6,
                anchor_xy[1] - 6,
                anchor_xy[0] + 6,
                anchor_xy[1] + 6,
                fill='',
                outline='#facc15',
                width=2,
            )
            if self._goal_pick_current_world is not None:
                current_xy = project_points_with_transform([self._goal_pick_current_world], transform)[0]
                canvas.create_line(
                    anchor_xy[0],
                    anchor_xy[1],
                    current_xy[0],
                    current_xy[1],
                    fill='#facc15',
                    width=2,
                    dash=(6, 4),
                )
        if not map_points and not boundary and not path and not current:
            canvas.create_text(width / 2, height / 2, text='等待行走规划 / 地图层 / 边界数据', fill='#cbd5e1', font=('DejaVu Sans', 16, 'bold'))
        canvas.create_text(
            14,
            14,
            anchor='nw',
            text='灰: map  红: global_costmap  蓝: local_costmap  蓝线: plan  橙线: 边界  黄: 点选目标',
            fill='#e5e7eb',
            font=('DejaVu Sans', 10),
        )

    def _draw_mobility_track_canvas(self, snapshot: dict[str, Any]) -> None:
        canvas = self.mobility_track_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 220)
        history = snapshot['track_history']
        if not history:
            canvas.create_text(width / 2, height / 2, text='等待 /cmd_vel', fill='#6b7280', font=('DejaVu Sans', 14, 'bold'))
            return
        now = history[-1][0]
        recent = [item for item in history if now - item[0] <= 30.0]
        left_values = [item[1] for item in recent]
        right_values = [item[2] for item in recent]
        min_value = min(left_values + right_values)
        max_value = max(left_values + right_values)
        for color, label, values in (
            ('#2563eb', 'Left Track', left_values),
            ('#dc2626', 'Right Track', right_values),
        ):
            projected = project_series(values, width, height, value_range=(min_value, max_value))
            if len(projected) >= 2:
                canvas.create_line(*[value for point in projected for value in point], fill=color, width=2, smooth=True)
            canvas.create_text(width - 12, 16 + (18 if label == 'Right Track' else 0), anchor='e', text=label, fill=color, font=('DejaVu Sans', 10, 'bold'))
        canvas.create_text(14, 14, anchor='nw', text='左右履带速度 (近 30s)', fill='#4b5563', font=('DejaVu Sans', 11, 'bold'))

    def _refresh_excavation(self, snapshot: dict[str, Any]) -> None:
        self.excavation_info_vars['phase'].set(snapshot['dig_status']['phase'])
        self.excavation_info_vars['detail'].set(snapshot['dig_status']['detail'])
        self.excavation_info_vars['swing'].set(f"{snapshot['swing_angle_deg']:.1f} deg")
        dig_debug: DigDebugState = snapshot['dig_debug']
        self.excavation_info_vars['files'].set(
            f"{dig_debug.summary} / seg3={len(dig_debug.seg3)} pts / cand={len(dig_debug.optimization_candidate)}"
        )
        if len(dig_debug.optimization_metrics) >= 4:
            self.excavation_info_vars['optimizer'].set(
                'eval='
                f"{int(dig_debug.optimization_metrics[0])} "
                f"obj={dig_debug.optimization_metrics[1]:.3f} "
                f"fill={dig_debug.optimization_metrics[2]:.2f} "
                f"tf={dig_debug.optimization_metrics[3]:.2f}"
            )
        else:
            self.excavation_info_vars['optimizer'].set('eval=0 obj=-- fill=--')
        self._draw_excavation_xy_canvas(dig_debug)
        self._draw_excavation_speed_canvas(dig_debug)
        self._draw_excavation_rotation_canvas(dig_debug)

    def _draw_excavation_xy_canvas(self, dig_debug: DigDebugState) -> None:
        canvas = self.excavation_xy_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 320)
        seg1_xy = [(x_value, y_value) for x_value, y_value, _ in dig_debug.seg1]
        seg2_xy = [(x_value, y_value) for x_value, y_value, _ in dig_debug.seg2]
        seg3_xy = [(x_value, y_value) for x_value, y_value, _ in dig_debug.seg3]
        candidate_xy = [(x_value, y_value) for x_value, y_value, _ in dig_debug.optimization_candidate]
        bounds = compute_bounds([seg1_xy, seg2_xy, seg3_xy, candidate_xy])
        transform = compute_view_transform(bounds, width, height)
        for color, points, label in (
            ('#2563eb', seg1_xy, 'Seg1'),
            ('#dc2626', seg2_xy, 'Seg2'),
            ('#059669', seg3_xy, 'Seg3'),
        ):
            if len(points) >= 2:
                projected = project_points_with_transform(downsample_points(points, max_points=320), transform)
                canvas.create_line(*[value for point in projected for value in point], fill=color, width=3, smooth=True)
                canvas.create_text(projected[-1][0] + 8, projected[-1][1], anchor='w', text=label, fill=color, font=('DejaVu Sans', 10, 'bold'))
        if len(candidate_xy) >= 2:
            projected = project_points_with_transform(downsample_points(candidate_xy, max_points=320), transform)
            canvas.create_line(
                *[value for point in projected for value in point],
                fill='#c026d3',
                width=2,
                dash=(8, 6),
                smooth=True,
            )
            canvas.create_text(projected[-1][0] + 8, projected[-1][1] + 12, anchor='w', text='Opt Candidate', fill='#c026d3', font=('DejaVu Sans', 10, 'bold'))
        if not seg1_xy and not seg2_xy and not seg3_xy and not candidate_xy:
            canvas.create_text(width / 2, height / 2, text='等待 digging/debug/segment*_path', fill='#6b7280', font=('DejaVu Sans', 16, 'bold'))
        canvas.create_text(14, 14, anchor='nw', text='挖掘轨迹平面视图 (实时, 含优化中间态)', fill='#4b5563', font=('DejaVu Sans', 11, 'bold'))

    def _draw_excavation_speed_canvas(self, dig_debug: DigDebugState) -> None:
        canvas = self.excavation_speed_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 220)
        series_map = {
            'Push Speed': dig_debug.vgan_result,
            'Hoist Speed': dig_debug.vrope_result,
        }
        all_values = [value for values in series_map.values() for value in values]
        if not all_values:
            canvas.create_text(width / 2, height / 2, text='等待 digging/debug/vgan_result', fill='#6b7280', font=('DejaVu Sans', 14, 'bold'))
            return
        min_value = min(all_values)
        max_value = max(all_values)
        canvas.create_text(14, 14, anchor='nw', text='推压/提升速度 (实时)', fill='#4b5563', font=('DejaVu Sans', 11, 'bold'))
        for color, (label, values) in zip(('#7c3aed', '#0f766e'), series_map.items()):
            projected = project_series(values, width, height, value_range=(min_value, max_value))
            if len(projected) >= 2:
                canvas.create_line(*[value for point in projected for value in point], fill=color, width=2, smooth=True)
            canvas.create_text(width - 18, 18 + (18 if label == 'Hoist Speed' else 0), anchor='e', text=label, fill=color, font=('DejaVu Sans', 10, 'bold'))

    def _draw_excavation_rotation_canvas(self, dig_debug: DigDebugState) -> None:
        canvas = self.excavation_rotation_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 220)
        series_map = {
            'Load': dig_debug.load_rotation_deg,
            'Return': dig_debug.return_rotation_deg,
        }
        all_values = [value for values in series_map.values() for value in values]
        if not all_values:
            canvas.create_text(width / 2, height / 2, text='等待 load/return 实时曲线', fill='#6b7280', font=('DejaVu Sans', 14, 'bold'))
            return
        min_value = min(all_values)
        max_value = max(all_values)
        canvas.create_text(14, 14, anchor='nw', text='装载/复位回转角 (deg)', fill='#4b5563', font=('DejaVu Sans', 11, 'bold'))
        for color, (label, values) in zip(('#b45309', '#1d4ed8'), series_map.items()):
            projected = project_series(values, width, height, value_range=(min_value, max_value))
            if len(projected) >= 2:
                canvas.create_line(*[value for point in projected for value in point], fill=color, width=2, smooth=True)
            canvas.create_text(width - 18, 18 + (18 if label == 'Return' else 0), anchor='e', text=label, fill=color, font=('DejaVu Sans', 10, 'bold'))

    @staticmethod
    def _format_pose(value: tuple[float, float, float] | None) -> str:
        if value is None:
            return '--'
        return f'x={value[0]:.2f}, y={value[1]:.2f}, yaw={value[2]:.1f}'


def main() -> None:
    rclpy.init()
    backend = MissionOperatorBackend()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        root = tk.Tk()
    except tk.TclError as exc:  # pragma: no cover
        print(f'Unable to start Tk UI: {exc}')
        executor.shutdown()
        backend.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    app = IntegratedOperatorHmi(root, backend)

    def on_close() -> None:
        root.destroy()
        executor.shutdown()
        backend.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
