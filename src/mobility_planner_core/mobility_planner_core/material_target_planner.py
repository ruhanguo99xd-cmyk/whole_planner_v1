from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from integrated_mission_interfaces.srv import ComputeMaterialTarget, ExtractMaterialBoundary
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mobility_planner_core.material_target_input import resolve_material_outline
from mobility_planner_core.material_target_layers import (
    DEFAULT_EDGE_NEAR_THRESHOLD_M,
    DEFAULT_MAX_CANDIDATES,
    DEFAULT_BOUNDARY_GRID_SIZE_M,
    DEFAULT_BOUNDARY_SAMPLE_COUNT,
    BoundaryFitConfig,
    CandidateWeights,
    CircularObstacle,
    SlopeZone,
    WorkBandConfig,
    evaluate_work_band,
    fit_material_boundary,
    generate_work_band,
)


DEFAULT_STANDOFF_M = 5.0


def _safe_json_loads(raw_text: str) -> dict:
    if not raw_text.strip():
        return {}
    return json.loads(raw_text)


def _dict_section(container: dict, key: str) -> dict:
    value = container.get(key, {})
    return value if isinstance(value, dict) else {}


def _resolve_numeric(
    *,
    field: str,
    default: float,
    profile_section: dict,
    constraint_section: dict,
    profile: dict,
    constraints: dict,
) -> float:
    return float(
        constraints.get(
            field,
            constraint_section.get(
                field,
                profile.get(
                    field,
                    profile_section.get(field, default),
                ),
            ),
        )
    )


def _resolve_standoff(desired_standoff_m: float, profile: dict, constraints: dict) -> float:
    standoff_m = desired_standoff_m if desired_standoff_m > 0.0 else DEFAULT_STANDOFF_M
    standoff_m = float(profile.get('preferred_standoff_m', standoff_m))
    standoff_m = float(constraints.get('override_standoff_m', standoff_m))
    return standoff_m


def _resolve_boundary_fit_config(profile: dict, constraints: dict) -> BoundaryFitConfig:
    profile_config = _dict_section(profile, 'boundary_fit')
    constraint_config = _dict_section(constraints, 'boundary_fit')
    return BoundaryFitConfig(
        grid_size_m=float(
            constraint_config.get(
                'grid_size_m',
                profile_config.get('grid_size_m', DEFAULT_BOUNDARY_GRID_SIZE_M),
            )
        ),
        min_points_per_cell=int(
            constraint_config.get(
                'min_points_per_cell',
                profile_config.get('min_points_per_cell', 1),
            )
        ),
        sample_count=int(
            constraint_config.get(
                'sample_count',
                profile_config.get('sample_count', DEFAULT_BOUNDARY_SAMPLE_COUNT),
            )
        ),
    )


def _parse_circular_obstacles(raw_items: object) -> tuple[CircularObstacle, ...]:
    if not isinstance(raw_items, list):
        return ()
    obstacles: list[CircularObstacle] = []
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        try:
            obstacles.append(
                CircularObstacle(
                    x=float(item.get('x', 0.0)),
                    y=float(item.get('y', 0.0)),
                    radius_m=max(0.0, float(item.get('radius_m', 0.0))),
                )
            )
        except (TypeError, ValueError):
            continue
    return tuple(obstacles)


def _parse_slope_zones(raw_items: object) -> tuple[SlopeZone, ...]:
    if not isinstance(raw_items, list):
        return ()
    zones: list[SlopeZone] = []
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        try:
            zones.append(
                SlopeZone(
                    x=float(item.get('x', 0.0)),
                    y=float(item.get('y', 0.0)),
                    radius_m=max(0.0, float(item.get('radius_m', 0.0))),
                    slope_deg=max(0.0, float(item.get('slope_deg', 0.0))),
                )
            )
        except (TypeError, ValueError):
            continue
    return tuple(zones)


def _resolve_work_band_config(
    *,
    standoff_m: float,
    profile: dict,
    constraints: dict,
    edge_near_threshold_m: float,
    max_candidates: int,
) -> WorkBandConfig:
    profile_config = _dict_section(profile, 'work_band')
    constraint_config = _dict_section(constraints, 'work_band')
    preferred_offset_m = float(
        constraint_config.get(
            'preferred_offset_m',
            profile_config.get('preferred_offset_m', standoff_m),
        )
    )
    min_offset_m = float(
        constraint_config.get(
            'min_offset_m',
            profile_config.get('min_offset_m', max(0.5, preferred_offset_m * 0.7)),
        )
    )
    max_offset_m = float(
        constraint_config.get(
            'max_offset_m',
            profile_config.get('max_offset_m', max(preferred_offset_m + 1.0, preferred_offset_m * 1.3)),
        )
    )
    if min_offset_m > preferred_offset_m:
        min_offset_m = preferred_offset_m
    if max_offset_m < preferred_offset_m:
        max_offset_m = preferred_offset_m
    obstacles = _parse_circular_obstacles(
        constraint_config.get(
            'obstacles',
            profile_config.get('obstacles', []),
        )
    )
    slope_zones = _parse_slope_zones(
        constraint_config.get(
            'slope_zones',
            profile_config.get('slope_zones', []),
        )
    )
    return WorkBandConfig(
        min_offset_m=min_offset_m,
        preferred_offset_m=preferred_offset_m,
        max_offset_m=max_offset_m,
        edge_near_threshold_m=edge_near_threshold_m,
        max_candidates=max_candidates,
        min_approach_distance_m=float(
            constraint_config.get(
                'min_approach_distance_m',
                profile_config.get('min_approach_distance_m', 0.0),
            )
        ),
        max_approach_distance_m=float(
            constraint_config.get(
                'max_approach_distance_m',
                profile_config.get('max_approach_distance_m', 50.0),
            )
        ),
        max_heading_change_deg=float(
            constraint_config.get(
                'max_heading_change_deg',
                profile_config.get('max_heading_change_deg', 135.0),
            )
        ),
        max_boundary_curvature=float(
            constraint_config.get(
                'max_boundary_curvature',
                profile_config.get('max_boundary_curvature', 1.5),
            )
        ),
        min_turn_radius_m=float(
            constraint_config.get(
                'min_turn_radius_m',
                profile_config.get('min_turn_radius_m', 0.0),
            )
        ),
        obstacle_clearance_m=float(
            constraint_config.get(
                'obstacle_clearance_m',
                profile_config.get('obstacle_clearance_m', 0.0),
            )
        ),
        max_slope_deg=float(
            constraint_config.get(
                'max_slope_deg',
                profile_config.get('max_slope_deg', 90.0),
            )
        ),
        preferred_yaw_offset_deg=float(
            constraint_config.get(
                'preferred_yaw_offset_deg',
                profile_config.get('preferred_yaw_offset_deg', 0.0),
            )
        ),
        obstacles=obstacles,
        slope_zones=slope_zones,
    )


def _resolve_candidate_weights(profile: dict, constraints: dict) -> CandidateWeights:
    profile_config = _dict_section(profile, 'candidate_evaluation')
    constraint_config = _dict_section(constraints, 'candidate_evaluation')
    return CandidateWeights(
        path_weight=_resolve_numeric(
            field='path_weight',
            default=1.0,
            profile_section=profile_config,
            constraint_section=constraint_config,
            profile=profile,
            constraints=constraints,
        ),
        reference_weight=_resolve_numeric(
            field='reference_weight',
            default=0.35,
            profile_section=profile_config,
            constraint_section=constraint_config,
            profile=profile,
            constraints=constraints,
        ),
        yaw_weight=_resolve_numeric(
            field='yaw_weight',
            default=0.2,
            profile_section=profile_config,
            constraint_section=constraint_config,
            profile=profile,
            constraints=constraints,
        ),
        distance_weight=_resolve_numeric(
            field='distance_weight',
            default=0.25,
            profile_section=profile_config,
            constraint_section=constraint_config,
            profile=profile,
            constraints=constraints,
        ),
        safety_weight=_resolve_numeric(
            field='safety_weight',
            default=0.8,
            profile_section=profile_config,
            constraint_section=constraint_config,
            profile=profile,
            constraints=constraints,
        ),
        curvature_weight=_resolve_numeric(
            field='curvature_weight',
            default=0.2,
            profile_section=profile_config,
            constraint_section=constraint_config,
            profile=profile,
            constraints=constraints,
        ),
    )


def _boundary_input_section(profile: dict, constraints: dict) -> tuple[dict, dict]:
    profile_section = profile.get('boundary_input', {})
    if not isinstance(profile_section, dict):
        profile_section = {}
    constraint_section = constraints.get('boundary_input', {})
    if not isinstance(constraint_section, dict):
        constraint_section = {}
    return profile_section, constraint_section


def should_use_pointcloud_extractor(
    *,
    material_outline: list[Point] | tuple[Point, ...],
    material_profile_json: str,
    planner_constraints_json: str,
) -> bool:
    if material_outline:
        return False
    profile = _safe_json_loads(material_profile_json)
    constraints = _safe_json_loads(planner_constraints_json)
    profile_section, constraint_section = _boundary_input_section(profile, constraints)
    use_extractor = constraint_section.get('use_pointcloud_extractor', profile_section.get('use_pointcloud_extractor', False))
    source = str(
        constraints.get(
            'geometry_source',
            constraint_section.get(
                'source',
                profile.get('geometry_source', profile_section.get('source', 'auto')),
            ),
        )
    ).strip().lower()
    return bool(use_extractor) or source in {'pointcloud', 'point_cloud', 'pointcloud_live', 'extracted_boundary'}


def compute_material_target(
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
    material_outline: list[Point],
    desired_standoff_m: float,
    material_profile_json: str = '',
    planner_constraints_json: str = '',
) -> tuple[PoseStamped, dict]:
    profile = _safe_json_loads(material_profile_json)
    constraints = _safe_json_loads(planner_constraints_json)
    resolved_outline, geometry_input_debug = resolve_material_outline(
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
        material_outline=material_outline,
        profile=profile,
        constraints=constraints,
    )
    standoff_m = _resolve_standoff(desired_standoff_m, profile, constraints)
    profile_work_band = _dict_section(profile, 'work_band')
    constraint_work_band = _dict_section(constraints, 'work_band')
    edge_near_threshold_m = float(
        constraints.get(
            'edge_near_threshold_m',
            constraint_work_band.get(
                'edge_near_threshold_m',
                profile.get(
                    'edge_near_threshold_m',
                    profile_work_band.get('edge_near_threshold_m', max(DEFAULT_EDGE_NEAR_THRESHOLD_M, standoff_m * 0.8)),
                ),
            ),
        )
    )
    max_candidates = int(
        constraints.get(
            'max_candidates',
            constraint_work_band.get(
                'max_candidates',
                profile.get('max_candidates', profile_work_band.get('max_candidates', DEFAULT_MAX_CANDIDATES)),
            ),
        )
    )
    weights = _resolve_candidate_weights(profile, constraints)
    strategy_hint = str(constraints.get('force_strategy', profile.get('target_strategy', 'auto'))).strip().lower()
    boundary_fit_config = _resolve_boundary_fit_config(profile, constraints)
    work_band_config = _resolve_work_band_config(
        standoff_m=standoff_m,
        profile=profile,
        constraints=constraints,
        edge_near_threshold_m=edge_near_threshold_m,
        max_candidates=max_candidates,
    )

    boundary = fit_material_boundary(
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
        material_outline=resolved_outline,
        fit_config=boundary_fit_config,
    )
    work_band = generate_work_band(
        boundary=boundary,
        config=work_band_config,
        strategy_hint=strategy_hint,
    )
    target, debug = evaluate_work_band(
        boundary=boundary,
        work_band=work_band,
        weights=weights,
    )
    debug.update(
        {
            'planner': 'material_target_planner_v4',
            'layers': ['boundary_fit', 'work_band', 'candidate_evaluation'],
            'profile_keys': sorted(profile.keys()),
            'constraint_keys': sorted(constraints.keys()),
            'geometry_input': geometry_input_debug,
            'boundary_fit_config': {
                'grid_size_m': boundary_fit_config.grid_size_m,
                'min_points_per_cell': boundary_fit_config.min_points_per_cell,
                'sample_count': boundary_fit_config.sample_count,
            },
            'work_band_config': {
                'min_offset_m': work_band_config.min_offset_m,
                'preferred_offset_m': work_band_config.preferred_offset_m,
                'max_offset_m': work_band_config.max_offset_m,
                'max_candidates': work_band_config.max_candidates,
                'min_approach_distance_m': work_band_config.min_approach_distance_m,
                'max_approach_distance_m': work_band_config.max_approach_distance_m,
                'max_heading_change_deg': work_band_config.max_heading_change_deg,
                'max_boundary_curvature': work_band_config.max_boundary_curvature,
                'min_turn_radius_m': work_band_config.min_turn_radius_m,
                'obstacle_clearance_m': work_band_config.obstacle_clearance_m,
                'max_slope_deg': work_band_config.max_slope_deg,
                'preferred_yaw_offset_deg': work_band_config.preferred_yaw_offset_deg,
                'obstacle_count': len(work_band_config.obstacles),
                'slope_zone_count': len(work_band_config.slope_zones),
            },
            'candidate_weight_config': {
                'path_weight': weights.path_weight,
                'reference_weight': weights.reference_weight,
                'distance_weight': weights.distance_weight,
                'safety_weight': weights.safety_weight,
                'curvature_weight': weights.curvature_weight,
                'yaw_weight': weights.yaw_weight,
            },
        }
    )
    return target, debug


# Backward-compatible alias for earlier tests and callers.
def compute_placeholder_target(
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
    material_outline: list[Point],
    desired_standoff_m: float,
    material_profile_json: str = '',
    planner_constraints_json: str = '',
) -> tuple[PoseStamped, dict]:
    return compute_material_target(
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
        material_outline=material_outline,
        desired_standoff_m=desired_standoff_m,
        material_profile_json=material_profile_json,
        planner_constraints_json=planner_constraints_json,
    )


class MaterialTargetPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('material_target_planner')
        self._callback_group = ReentrantCallbackGroup()
        self.declare_parameter('service_name', '/mobility/compute_material_target')
        self.declare_parameter('enable_boundary_extractor', True)
        self.declare_parameter('boundary_extractor_service_name', '/mobility/extract_material_boundary')
        self.declare_parameter('boundary_extractor_timeout_sec', 2.0)
        self._boundary_extractor_enabled = bool(self.get_parameter('enable_boundary_extractor').value)
        self._boundary_extractor_client = self.create_client(
            ExtractMaterialBoundary,
            str(self.get_parameter('boundary_extractor_service_name').value),
            callback_group=self._callback_group,
        )
        self.create_service(
            ComputeMaterialTarget,
            str(self.get_parameter('service_name').value),
            self._handle_request,
            callback_group=self._callback_group,
        )
        self.get_logger().info('material_target_planner started')

    def _request_extracted_boundary(self, request) -> tuple[list[Point], dict]:
        if not self._boundary_extractor_enabled:
            raise RuntimeError('boundary extractor disabled')
        timeout_sec = float(self.get_parameter('boundary_extractor_timeout_sec').value)
        if not self._boundary_extractor_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('boundary extractor service unavailable')

        extractor_request = ExtractMaterialBoundary.Request()
        extractor_request.request_id = request.request_id
        extractor_request.current_pose = request.current_pose
        extractor_request.material_reference_pose = request.material_reference_pose
        extractor_request.material_profile_json = request.material_profile_json
        extractor_request.planner_constraints_json = request.planner_constraints_json
        future = self._boundary_extractor_client.call_async(extractor_request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError('boundary extractor request timed out')
        result = future.result()
        if result is None:
            raise RuntimeError('boundary extractor returned no response')
        debug = _safe_json_loads(result.debug_json)
        if not result.success:
            message = result.message or 'boundary extractor failed'
            raise RuntimeError(message)
        return list(result.boundary_outline), debug

    def _handle_request(self, request, response):
        extracted_outline_debug = None
        material_outline = list(request.material_outline)
        if should_use_pointcloud_extractor(
            material_outline=material_outline,
            material_profile_json=request.material_profile_json,
            planner_constraints_json=request.planner_constraints_json,
        ):
            try:
                material_outline, extracted_outline_debug = self._request_extracted_boundary(request)
            except Exception as exc:
                response.success = False
                response.message = f'point cloud boundary extraction failed: {exc}'
                response.walk_constraints_json = '{}'
                response.debug_json = json.dumps(
                    {'planner': 'material_target_planner_v4', 'error': str(exc), 'stage': 'boundary_extractor'},
                    ensure_ascii=True,
                )
                return response
        try:
            target, debug = compute_material_target(
                current_pose=request.current_pose,
                material_reference_pose=request.material_reference_pose,
                material_outline=material_outline,
                desired_standoff_m=float(request.desired_standoff_m),
                material_profile_json=request.material_profile_json,
                planner_constraints_json=request.planner_constraints_json,
            )
        except Exception as exc:
            response.success = False
            response.message = f'material target planning failed: {exc}'
            response.walk_constraints_json = '{}'
            response.debug_json = json.dumps({'planner': 'material_target_planner_v4', 'error': str(exc)}, ensure_ascii=True)
            return response
        if extracted_outline_debug is not None:
            geometry_debug = debug.get('geometry_input', {})
            geometry_debug['upstream_source'] = geometry_debug.get('source')
            geometry_debug['source'] = 'pointcloud_boundary_extractor'
            geometry_debug['extracted_outline_count'] = len(material_outline)
            debug['geometry_input'] = geometry_debug
            debug['pointcloud_boundary_extractor'] = extracted_outline_debug
        response.success = True
        response.message = 'received'
        response.target_pose = target
        response.walk_constraints_json = request.planner_constraints_json or '{}'
        response.debug_json = json.dumps(debug, ensure_ascii=True, separators=(',', ':'))
        return response


def main() -> None:
    rclpy.init()
    node = MaterialTargetPlannerNode()
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
