from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped
from integrated_mission_interfaces.srv import ExtractMaterialBoundary
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from mobility_planner_core.material_target_input import (
    DEFAULT_MIN_BOUNDARY_RADIUS_M,
    DEFAULT_SCATTER_ANGULAR_BINS,
    extract_outline_from_scatter_points,
)


@dataclass(frozen=True)
class PointCloudBoundaryConfig:
    z_min_m: float = -math.inf
    z_max_m: float = math.inf
    roi_radius_m: float = 0.0
    angular_bins: int = DEFAULT_SCATTER_ANGULAR_BINS
    min_boundary_radius_m: float = DEFAULT_MIN_BOUNDARY_RADIUS_M
    max_input_points: int = 0
    min_points_required: int = 8


@dataclass(frozen=True)
class StaticExtrinsicConfig:
    enabled: bool = False
    sensor_frame_id: str = ''
    translation_x_m: float = 0.0
    translation_y_m: float = 0.0
    translation_z_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


def _safe_json_loads(raw_text: str) -> dict:
    if not raw_text.strip():
        return {}
    return json.loads(raw_text)


def _dict_section(container: dict, keys: tuple[str, ...]) -> dict:
    for key in keys:
        value = container.get(key, {})
        if isinstance(value, dict):
            return value
    return {}


def _point_from_object(raw: object) -> Point | None:
    if isinstance(raw, Point):
        return Point(x=raw.x, y=raw.y, z=raw.z)
    if isinstance(raw, dict):
        try:
            return Point(x=float(raw.get('x', 0.0)), y=float(raw.get('y', 0.0)), z=float(raw.get('z', 0.0)))
        except (TypeError, ValueError):
            return None
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        try:
            return Point(x=float(raw[0]), y=float(raw[1]), z=float(raw[2]) if len(raw) >= 3 else 0.0)
        except (TypeError, ValueError):
            return None
    return None


def _resolve_named_or_point(
    raw: object,
    *,
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
) -> tuple[float, float, str]:
    if isinstance(raw, str):
        value = raw.strip().lower()
        if value in {'current', 'current_pose'}:
            return current_pose.pose.position.x, current_pose.pose.position.y, 'current_pose'
        if value in {'material_reference', 'reference', 'material_reference_pose'}:
            return material_reference_pose.pose.position.x, material_reference_pose.pose.position.y, 'material_reference_pose'
    point = _point_from_object(raw)
    if point is not None:
        return point.x, point.y, 'explicit_point'
    return material_reference_pose.pose.position.x, material_reference_pose.pose.position.y, 'material_reference_pose'


def _resolve_target_frame(current_pose: PoseStamped, material_reference_pose: PoseStamped) -> str:
    current_frame = current_pose.header.frame_id.strip()
    reference_frame = material_reference_pose.header.frame_id.strip()
    if current_frame and reference_frame and current_frame != reference_frame:
        raise ValueError(f'pose frame mismatch: current_pose={current_frame}, material_reference_pose={reference_frame}')
    return reference_frame or current_frame or 'map'


def _resolve_pointcloud_config(
    *,
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
    profile: dict,
    constraints: dict,
) -> tuple[PointCloudBoundaryConfig, dict]:
    input_config = _dict_section(profile, ('boundary_input', 'geometry_input', 'point_cloud_input'))
    input_constraints = _dict_section(constraints, ('boundary_input', 'geometry_input', 'point_cloud_input'))

    roi_center_x, roi_center_y, roi_center_source = _resolve_named_or_point(
        input_constraints.get('roi_center', input_config.get('roi_center', 'material_reference_pose')),
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
    )
    centroid_hint_x, centroid_hint_y, centroid_hint_source = _resolve_named_or_point(
        input_constraints.get('centroid_hint', input_config.get('centroid_hint', 'material_reference_pose')),
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
    )
    config = PointCloudBoundaryConfig(
        z_min_m=float(input_constraints.get('z_min_m', input_config.get('z_min_m', -math.inf))),
        z_max_m=float(input_constraints.get('z_max_m', input_config.get('z_max_m', math.inf))),
        roi_radius_m=float(input_constraints.get('roi_radius_m', input_config.get('roi_radius_m', 0.0))),
        angular_bins=max(8, int(input_constraints.get('angular_bins', input_config.get('angular_bins', DEFAULT_SCATTER_ANGULAR_BINS)))),
        min_boundary_radius_m=max(
            0.0,
            float(input_constraints.get('min_boundary_radius_m', input_config.get('min_boundary_radius_m', DEFAULT_MIN_BOUNDARY_RADIUS_M))),
        ),
        max_input_points=max(0, int(input_constraints.get('max_input_points', input_config.get('max_input_points', 0)))),
        min_points_required=max(3, int(input_constraints.get('min_points_required', input_config.get('min_points_required', 8)))),
    )
    debug = {
        'roi_center_x': roi_center_x,
        'roi_center_y': roi_center_y,
        'roi_center_source': roi_center_source,
        'centroid_hint_x': centroid_hint_x,
        'centroid_hint_y': centroid_hint_y,
        'centroid_hint_source': centroid_hint_source,
        'z_min_m': config.z_min_m,
        'z_max_m': config.z_max_m,
        'roi_radius_m': config.roi_radius_m,
        'angular_bins': config.angular_bins,
        'min_boundary_radius_m': config.min_boundary_radius_m,
        'max_input_points': config.max_input_points,
        'min_points_required': config.min_points_required,
    }
    return config, debug


def _rotation_matrix_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        ((cy * cp), (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr)),
        ((sy * cp), (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr)),
        (-sp, cp * sr, cp * cr),
    )


def _apply_rotation(matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]], x: float, y: float, z: float) -> tuple[float, float, float]:
    return (
        (matrix[0][0] * x) + (matrix[0][1] * y) + (matrix[0][2] * z),
        (matrix[1][0] * x) + (matrix[1][1] * y) + (matrix[1][2] * z),
        (matrix[2][0] * x) + (matrix[2][1] * y) + (matrix[2][2] * z),
    )


def _resolve_static_extrinsic(profile: dict, constraints: dict, node_config: StaticExtrinsicConfig | None = None) -> StaticExtrinsicConfig:
    input_config = _dict_section(profile, ('boundary_input', 'geometry_input', 'point_cloud_input'))
    input_constraints = _dict_section(constraints, ('boundary_input', 'geometry_input', 'point_cloud_input'))
    static_config = input_constraints.get('static_extrinsic', input_config.get('static_extrinsic', {}))
    if not isinstance(static_config, dict):
        static_config = {}

    base = node_config or StaticExtrinsicConfig()
    return StaticExtrinsicConfig(
        enabled=bool(static_config.get('enabled', base.enabled)),
        sensor_frame_id=str(static_config.get('sensor_frame_id', base.sensor_frame_id)).strip(),
        translation_x_m=float(static_config.get('translation_x_m', base.translation_x_m)),
        translation_y_m=float(static_config.get('translation_y_m', base.translation_y_m)),
        translation_z_m=float(static_config.get('translation_z_m', base.translation_z_m)),
        roll_deg=float(static_config.get('roll_deg', base.roll_deg)),
        pitch_deg=float(static_config.get('pitch_deg', base.pitch_deg)),
        yaw_deg=float(static_config.get('yaw_deg', base.yaw_deg)),
    )


def transform_xyz_points_to_target_frame(
    *,
    xyz_points: Iterable[tuple[float, float, float]],
    cloud_frame_id: str,
    target_frame_id: str,
    static_extrinsic: StaticExtrinsicConfig,
) -> tuple[tuple[tuple[float, float, float], ...], dict]:
    xyz_points = tuple(xyz_points)
    normalized_cloud_frame = cloud_frame_id.strip() or target_frame_id.strip() or 'map'
    normalized_target_frame = target_frame_id.strip() or normalized_cloud_frame
    if normalized_cloud_frame == normalized_target_frame:
        return xyz_points, {
            'mode': 'identity_same_frame',
            'cloud_frame_id': normalized_cloud_frame,
            'target_frame_id': normalized_target_frame,
        }

    if not static_extrinsic.enabled:
        raise ValueError(
            f'frame mismatch without tf or static extrinsic: cloud_frame={normalized_cloud_frame}, target_frame={normalized_target_frame}'
        )
    if static_extrinsic.sensor_frame_id and normalized_cloud_frame != static_extrinsic.sensor_frame_id:
        raise ValueError(
            f'cloud frame {normalized_cloud_frame} does not match configured sensor_frame_id {static_extrinsic.sensor_frame_id}'
        )

    rotation = _rotation_matrix_from_rpy_deg(
        static_extrinsic.roll_deg,
        static_extrinsic.pitch_deg,
        static_extrinsic.yaw_deg,
    )
    transformed: list[tuple[float, float, float]] = []
    for x_value, y_value, z_value in xyz_points:
        rot_x, rot_y, rot_z = _apply_rotation(rotation, x_value, y_value, z_value)
        transformed.append(
            (
                rot_x + static_extrinsic.translation_x_m,
                rot_y + static_extrinsic.translation_y_m,
                rot_z + static_extrinsic.translation_z_m,
            )
        )
    debug = {
        'mode': 'static_extrinsic',
        'cloud_frame_id': normalized_cloud_frame,
        'target_frame_id': normalized_target_frame,
        'sensor_frame_id': static_extrinsic.sensor_frame_id or normalized_cloud_frame,
        'translation_m': {
            'x': static_extrinsic.translation_x_m,
            'y': static_extrinsic.translation_y_m,
            'z': static_extrinsic.translation_z_m,
        },
        'rotation_deg': {
            'roll': static_extrinsic.roll_deg,
            'pitch': static_extrinsic.pitch_deg,
            'yaw': static_extrinsic.yaw_deg,
        },
    }
    return tuple(transformed), debug


def extract_boundary_from_xyz_points(
    *,
    xyz_points: Iterable[tuple[float, float, float]],
    cloud_frame_id: str,
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
    profile: dict,
    constraints: dict,
    node_static_extrinsic: StaticExtrinsicConfig | None = None,
) -> tuple[tuple[Point, ...], dict]:
    xyz_points = tuple(xyz_points)
    target_frame_id = _resolve_target_frame(current_pose, material_reference_pose)
    config, config_debug = _resolve_pointcloud_config(
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
        profile=profile,
        constraints=constraints,
    )
    static_extrinsic = _resolve_static_extrinsic(profile, constraints, node_static_extrinsic)
    transformed_xyz_points, frame_transform_debug = transform_xyz_points_to_target_frame(
        xyz_points=xyz_points,
        cloud_frame_id=cloud_frame_id,
        target_frame_id=target_frame_id,
        static_extrinsic=static_extrinsic,
    )
    scatter_points: list[Point] = []
    z_filtered_count = 0
    for x_value, y_value, z_value in transformed_xyz_points:
        if z_value < config.z_min_m or z_value > config.z_max_m:
            continue
        z_filtered_count += 1
        scatter_points.append(Point(x=float(x_value), y=float(y_value), z=float(z_value)))

    outline, helper_debug = extract_outline_from_scatter_points(
        scatter_points,
        center_x=config_debug['centroid_hint_x'],
        center_y=config_debug['centroid_hint_y'],
        roi_center_x=config_debug['roi_center_x'],
        roi_center_y=config_debug['roi_center_y'],
        roi_radius_m=config.roi_radius_m,
        angular_bins=config.angular_bins,
        min_boundary_radius_m=config.min_boundary_radius_m,
        max_input_points=config.max_input_points,
    )
    debug = {
        'cloud_point_count': len(transformed_xyz_points),
        'z_filtered_count': z_filtered_count,
        'outline_point_count': len(outline),
        'config': config_debug,
        'frame_transform': frame_transform_debug,
        'scatter_outline': helper_debug,
    }
    if len(outline) < config.min_points_required:
        debug['status'] = 'insufficient_boundary_points'
    else:
        debug['status'] = 'ok'
    return outline, debug


class MaterialBoundaryExtractorNode(Node):
    def __init__(self) -> None:
        super().__init__('material_boundary_extractor')
        self.declare_parameter('point_cloud_topic', '/material/point_cloud')
        self.declare_parameter('service_name', '/mobility/extract_material_boundary')
        self.declare_parameter('boundary_topic', '/mobility/material_boundary')
        self.declare_parameter('debug_topic', '/mobility/material_boundary_debug')
        self.declare_parameter('use_static_extrinsic', False)
        self.declare_parameter('sensor_frame_id', '')
        self.declare_parameter('extrinsic_translation_x_m', 0.0)
        self.declare_parameter('extrinsic_translation_y_m', 0.0)
        self.declare_parameter('extrinsic_translation_z_m', 0.0)
        self.declare_parameter('extrinsic_roll_deg', 0.0)
        self.declare_parameter('extrinsic_pitch_deg', 0.0)
        self.declare_parameter('extrinsic_yaw_deg', 0.0)
        self._latest_cloud: PointCloud2 | None = None
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('point_cloud_topic').value),
            self._handle_cloud,
            10,
        )
        self._boundary_publisher = self.create_publisher(
            PolygonStamped,
            str(self.get_parameter('boundary_topic').value),
            10,
        )
        self._debug_publisher = self.create_publisher(
            String,
            str(self.get_parameter('debug_topic').value),
            10,
        )
        self.create_service(
            ExtractMaterialBoundary,
            str(self.get_parameter('service_name').value),
            self._handle_request,
        )
        self.get_logger().info('material_boundary_extractor started')

    def _node_static_extrinsic(self) -> StaticExtrinsicConfig:
        return StaticExtrinsicConfig(
            enabled=bool(self.get_parameter('use_static_extrinsic').value),
            sensor_frame_id=str(self.get_parameter('sensor_frame_id').value).strip(),
            translation_x_m=float(self.get_parameter('extrinsic_translation_x_m').value),
            translation_y_m=float(self.get_parameter('extrinsic_translation_y_m').value),
            translation_z_m=float(self.get_parameter('extrinsic_translation_z_m').value),
            roll_deg=float(self.get_parameter('extrinsic_roll_deg').value),
            pitch_deg=float(self.get_parameter('extrinsic_pitch_deg').value),
            yaw_deg=float(self.get_parameter('extrinsic_yaw_deg').value),
        )

    def _handle_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _publish_boundary(self, frame_id: str, outline: tuple[Point, ...]) -> None:
        polygon = PolygonStamped()
        polygon.header.frame_id = frame_id or 'map'
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.polygon.points = [Point32(x=point.x, y=point.y, z=point.z) for point in outline]
        self._boundary_publisher.publish(polygon)

    def _publish_debug(self, debug: dict) -> None:
        msg = String()
        msg.data = json.dumps(debug, ensure_ascii=True, separators=(',', ':'))
        self._debug_publisher.publish(msg)

    def _handle_request(self, request, response):
        if self._latest_cloud is None:
            response.success = False
            response.message = 'no point cloud received yet'
            response.debug_json = json.dumps({'status': 'no_point_cloud'}, ensure_ascii=True)
            return response

        profile = _safe_json_loads(request.material_profile_json)
        constraints = _safe_json_loads(request.planner_constraints_json)
        try:
            xyz_points = tuple(
                (float(x_value), float(y_value), float(z_value))
                for x_value, y_value, z_value in point_cloud2.read_points(
                    self._latest_cloud,
                    field_names=('x', 'y', 'z'),
                    skip_nans=True,
                )
            )
            outline, debug = extract_boundary_from_xyz_points(
                xyz_points=xyz_points,
                cloud_frame_id=self._latest_cloud.header.frame_id,
                current_pose=request.current_pose,
                material_reference_pose=request.material_reference_pose,
                profile=profile,
                constraints=constraints,
                node_static_extrinsic=self._node_static_extrinsic(),
            )
            debug['frame_id'] = self._latest_cloud.header.frame_id
        except Exception as exc:
            response.success = False
            response.message = f'boundary extraction failed: {exc}'
            response.debug_json = json.dumps({'status': 'error', 'error': str(exc)}, ensure_ascii=True)
            return response

        self._publish_debug(debug)
        if len(outline) < 3:
            response.success = False
            response.message = 'boundary extraction produced too few points'
            response.debug_json = json.dumps(debug, ensure_ascii=True, separators=(',', ':'))
            return response

        self._publish_boundary(self._latest_cloud.header.frame_id, outline)
        response.success = True
        response.message = 'received'
        response.boundary_outline = list(outline)
        response.debug_json = json.dumps(debug, ensure_ascii=True, separators=(',', ':'))
        return response


def main() -> None:
    rclpy.init()
    node = MaterialBoundaryExtractorNode()
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
