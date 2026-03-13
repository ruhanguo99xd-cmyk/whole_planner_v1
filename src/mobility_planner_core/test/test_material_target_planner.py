import json
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from integrated_mission_interfaces.srv import ComputeMaterialTarget

from mobility_planner_core.material_target_layers import (
    CandidateWeights,
    CircularObstacle,
    MaterialBoundaryModel,
    SlopeZone,
    WorkBandCandidate,
    WorkBandConfig,
    WorkBandPlan,
    evaluate_work_band,
    fit_material_boundary,
    generate_work_band,
)
from mobility_planner_core.material_target_planner import MaterialTargetPlannerNode, compute_material_target


def _pose(x: float, y: float, frame_id: str = 'map') -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def test_compute_material_target_uses_outline_center() -> None:
    current = _pose(0.0, 0.0)
    material = _pose(10.0, 0.0)
    outline = [Point(x=10.0, y=-1.0, z=0.0), Point(x=10.0, y=1.0, z=0.0)]

    target, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
        desired_standoff_m=5.0,
    )

    assert debug['work_band']['strategy'] == 'far_field'
    assert debug['candidate_evaluation']['candidate_source'] == 'current_approach'
    assert math.isclose(target.pose.position.x, 5.0, abs_tol=1e-6)
    assert math.isclose(target.pose.position.y, 0.0, abs_tol=1e-6)
    assert debug['boundary_fit']['centroid_source'] == 'outline_mean'
    assert debug['boundary_fit']['fit_method'] == 'degenerate_line_boundary'
    assert debug['boundary_fit']['point_count_input'] == 2
    assert debug['layers'] == ['boundary_fit', 'work_band', 'candidate_evaluation']
    assert debug['planner'] == 'material_target_planner_v4'


def test_compute_material_target_accepts_profile_override() -> None:
    current = _pose(0.0, 0.0)
    material = _pose(8.0, 0.0)

    target, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=[],
        desired_standoff_m=2.0,
        material_profile_json='{"preferred_standoff_m": 3.5}',
    )

    assert math.isclose(target.pose.position.x, 4.5, abs_tol=1e-6)
    assert math.isclose(debug['work_band']['preferred_offset_m'], 3.5, abs_tol=1e-6)


def test_compute_material_target_uses_edge_near_branch_when_close_to_outline() -> None:
    current = _pose(7.8, 0.1)
    material = _pose(10.0, 0.0)
    outline = [
        Point(x=8.0, y=0.0, z=0.0),
        Point(x=10.0, y=0.0, z=0.0),
        Point(x=12.0, y=0.0, z=0.0),
    ]

    target, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
        desired_standoff_m=2.0,
        planner_constraints_json='{"edge_near_threshold_m": 1.0}',
    )

    assert debug['work_band']['strategy'] == 'edge_near'
    assert math.isclose(target.pose.position.x, 6.0, abs_tol=1e-2)
    assert math.isclose(target.pose.position.y, 0.1, abs_tol=1e-1)


def test_compute_material_target_uses_far_field_branch_when_far_from_outline() -> None:
    current = _pose(0.0, 0.0)
    material = _pose(10.0, 0.0)
    outline = [
        Point(x=8.0, y=0.0, z=0.0),
        Point(x=10.0, y=0.0, z=0.0),
        Point(x=12.0, y=0.0, z=0.0),
    ]

    target, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
        desired_standoff_m=2.0,
        planner_constraints_json='{"edge_near_threshold_m": 0.5}',
    )

    assert debug['work_band']['strategy'] == 'far_field'
    assert debug['work_band']['candidate_count'] == 4
    assert debug['work_band']['kept_station_count'] > 0
    assert math.isclose(target.pose.position.x, 6.0, abs_tol=1e-6)
    assert debug['candidate_evaluation']['anchor_arc_length_m'] >= 0.0
    assert math.isclose(debug['candidate_evaluation']['target_offset_m'], 2.0, abs_tol=1e-6)


def test_work_band_layers_expose_boundary_and_candidate_generation() -> None:
    current = _pose(2.0, 0.0)
    material = _pose(10.0, 0.0)
    outline = [
        Point(x=8.0, y=-1.0, z=0.0),
        Point(x=8.0, y=1.0, z=0.0),
        Point(x=12.0, y=1.0, z=0.0),
        Point(x=12.0, y=-1.0, z=0.0),
    ]

    boundary = fit_material_boundary(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
    )
    work_band = generate_work_band(
        boundary=boundary,
        config=WorkBandConfig(
            min_offset_m=1.5,
            preferred_offset_m=2.5,
            max_offset_m=3.0,
            edge_near_threshold_m=0.5,
            max_candidates=3,
        ),
        strategy_hint='far_field',
    )

    assert boundary.centroid_source == 'outline_mean'
    assert boundary.fit_method == 'piecewise_cubic_boundary'
    assert boundary.point_count_samples >= len(outline)
    assert work_band.strategy == 'far_field'
    assert len(work_band.stations) == boundary.point_count_samples
    assert len(work_band.kept_station_indices) > 0
    assert len(work_band.candidates) == 4


def test_boundary_fit_reports_curve_metrics_and_nearest_arc() -> None:
    current = _pose(0.0, -3.0)
    material = _pose(0.0, 0.0)
    outline = [
        Point(x=-1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=1.0, z=0.0),
        Point(x=-1.0, y=1.0, z=0.0),
    ]

    boundary = fit_material_boundary(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
    )

    assert boundary.fit_method == 'piecewise_cubic_boundary'
    assert boundary.total_arc_length_m > 0.0
    assert boundary.nearest_arc_length_m >= 0.0
    assert boundary.fit_quality_mean_error_m >= 0.0
    assert boundary.closure_gap_m >= 0.0


def test_work_band_clips_unreachable_stations() -> None:
    current = _pose(-10.0, 0.0)
    material = _pose(0.0, 0.0)
    outline = [
        Point(x=-1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=1.0, z=0.0),
        Point(x=-1.0, y=1.0, z=0.0),
    ]

    boundary = fit_material_boundary(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
    )
    work_band = generate_work_band(
        boundary=boundary,
        config=WorkBandConfig(
            min_offset_m=2.0,
            preferred_offset_m=2.5,
            max_offset_m=3.0,
            max_candidates=6,
            max_approach_distance_m=9.0,
        ),
        strategy_hint='far_field',
    )

    assert len(work_band.stations) == boundary.point_count_samples
    assert 0 < len(work_band.kept_station_indices) < len(work_band.stations)


def test_compute_material_target_parses_nested_work_band_and_weight_config() -> None:
    current = _pose(-6.0, 0.0)
    material = _pose(0.0, 0.0)
    outline = [
        Point(x=-1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=1.0, z=0.0),
        Point(x=-1.0, y=1.0, z=0.0),
    ]

    _, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
        desired_standoff_m=2.5,
        material_profile_json=(
            '{"work_band":{"preferred_offset_m":2.8,"min_turn_radius_m":3.0},'
            '"candidate_evaluation":{"reference_weight":0.45}}'
        ),
        planner_constraints_json=(
            '{"work_band":{"obstacle_clearance_m":0.8,"max_slope_deg":9.0,'
            '"obstacles":[{"x":-3.8,"y":0.0,"radius_m":0.6}],'
            '"slope_zones":[{"x":-3.8,"y":0.0,"radius_m":1.0,"slope_deg":6.0}]},'
            '"candidate_evaluation":{"distance_weight":0.9,"safety_weight":1.4,"curvature_weight":0.5}}'
        ),
    )

    assert math.isclose(debug['work_band_config']['preferred_offset_m'], 2.8, abs_tol=1e-6)
    assert math.isclose(debug['work_band_config']['min_turn_radius_m'], 3.0, abs_tol=1e-6)
    assert math.isclose(debug['work_band_config']['obstacle_clearance_m'], 0.8, abs_tol=1e-6)
    assert math.isclose(debug['work_band_config']['max_slope_deg'], 9.0, abs_tol=1e-6)
    assert debug['work_band_config']['obstacle_count'] == 1
    assert debug['work_band_config']['slope_zone_count'] == 1
    assert math.isclose(debug['candidate_weight_config']['reference_weight'], 0.45, abs_tol=1e-6)
    assert math.isclose(debug['candidate_weight_config']['distance_weight'], 0.9, abs_tol=1e-6)
    assert math.isclose(debug['candidate_weight_config']['safety_weight'], 1.4, abs_tol=1e-6)
    assert math.isclose(debug['candidate_weight_config']['curvature_weight'], 0.5, abs_tol=1e-6)


def test_compute_material_target_uses_boundary_input_outline_points_when_outline_is_empty() -> None:
    current = _pose(0.0, -5.0)
    material = _pose(0.0, 0.0)

    _, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=[],
        desired_standoff_m=2.5,
        planner_constraints_json=(
            '{"boundary_input":{"source":"outline_points","outline_points":['
            '{"x":-1.0,"y":-1.0},{"x":1.0,"y":-1.0},{"x":1.0,"y":1.0},{"x":-1.0,"y":1.0}'
            ']}}'
        ),
    )

    assert debug['geometry_input']['source'] == 'outline_points'
    assert debug['geometry_input']['outline_point_count'] == 4
    assert debug['boundary_fit']['point_count_input'] == 4
    assert debug['boundary_fit']['fit_method'] == 'piecewise_cubic_boundary'


def test_compute_material_target_extracts_outline_from_scatter_points() -> None:
    current = _pose(0.0, -6.0)
    material = _pose(0.0, 0.0)
    scatter_points = [
        {"x": 0.0, "y": -2.0},
        {"x": 1.4, "y": -1.4},
        {"x": 2.0, "y": 0.0},
        {"x": 1.4, "y": 1.4},
        {"x": 0.0, "y": 2.0},
        {"x": -1.4, "y": 1.4},
        {"x": -2.0, "y": 0.0},
        {"x": -1.4, "y": -1.4},
        {"x": 0.0, "y": 0.0},
    ]

    _, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=[],
        desired_standoff_m=2.5,
        planner_constraints_json=(
            json.dumps(
                {
                    'boundary_input': {
                        'source': 'scatter_points',
                        'scatter_points': scatter_points,
                        'angular_bins': 16,
                        'min_boundary_radius_m': 0.5,
                    }
                }
            )
        ),
    )

    assert debug['geometry_input']['source'] == 'scatter_points'
    assert debug['geometry_input']['boundary_mode'] == 'scatter_boundary'
    assert debug['geometry_input']['outline_point_count'] >= 8
    assert debug['boundary_fit']['fit_method'] == 'piecewise_cubic_boundary'


def test_compute_material_target_uses_line_strips_as_boundary_input() -> None:
    current = _pose(-5.0, 0.0)
    material = _pose(0.0, 0.0)

    _, debug = compute_material_target(
        current_pose=current,
        material_reference_pose=material,
        material_outline=[],
        desired_standoff_m=2.5,
        planner_constraints_json=json.dumps(
            {
                'boundary_input': {
                    'source': 'line_strips',
                    'line_strips': [
                        [{'x': -1.0, 'y': -1.0}, {'x': 1.0, 'y': -1.0}],
                        [{'x': 1.0, 'y': 1.0}, {'x': -1.0, 'y': 1.0}],
                    ],
                }
            }
        ),
    )

    assert debug['geometry_input']['source'] == 'line_strips'
    assert debug['geometry_input']['line_strip_point_count'] == 4
    assert debug['boundary_fit']['point_count_input'] == 4


def test_work_band_marks_obstacle_and_slope_constrained_stations_infeasible() -> None:
    current = _pose(-6.0, 0.0)
    material = _pose(0.0, 0.0)
    outline = [
        Point(x=-1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=1.0, z=0.0),
        Point(x=-1.0, y=1.0, z=0.0),
    ]

    boundary = fit_material_boundary(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
    )
    work_band = generate_work_band(
        boundary=boundary,
        config=WorkBandConfig(
            min_offset_m=2.0,
            preferred_offset_m=2.5,
            max_offset_m=3.0,
            max_candidates=8,
            obstacle_clearance_m=0.9,
            max_slope_deg=5.0,
            obstacles=(CircularObstacle(x=-3.5, y=0.0, radius_m=0.6),),
            slope_zones=(SlopeZone(x=-3.5, y=0.0, radius_m=1.0, slope_deg=8.0),),
        ),
        strategy_hint='far_field',
    )

    reasons = {reason for station in work_band.stations for reason in station.feasibility_reasons}
    assert 'obstacle_clearance_too_small' in reasons
    assert 'local_slope_too_large' in reasons
    assert len(work_band.kept_station_indices) < len(work_band.stations)


def test_work_band_marks_small_turn_radius_stations_infeasible() -> None:
    current = _pose(0.0, -4.0)
    material = _pose(0.0, 0.0)
    outline = [
        Point(x=-1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=-1.0, z=0.0),
        Point(x=1.0, y=1.0, z=0.0),
        Point(x=-1.0, y=1.0, z=0.0),
    ]

    boundary = fit_material_boundary(
        current_pose=current,
        material_reference_pose=material,
        material_outline=outline,
    )
    work_band = generate_work_band(
        boundary=boundary,
        config=WorkBandConfig(
            min_offset_m=2.0,
            preferred_offset_m=2.5,
            max_offset_m=3.0,
            max_candidates=8,
            min_turn_radius_m=6.0,
        ),
        strategy_hint='far_field',
    )

    assert any('turn_radius_too_small' in station.feasibility_reasons for station in work_band.stations)


def test_candidate_evaluation_prefers_safer_candidate_when_safety_weight_is_high() -> None:
    boundary = MaterialBoundaryModel(
        frame_id='map',
        centroid_x=0.0,
        centroid_y=0.0,
        centroid_z=0.0,
        centroid_source='manual',
        current_x=-6.0,
        current_y=0.0,
        current_z=0.0,
        current_heading=0.0,
        reference_x=0.0,
        reference_y=0.0,
        reference_z=0.0,
        ordered_boundary_points=(),
        curve_samples=(),
        total_arc_length_m=0.0,
        nearest_edge_point=None,
        nearest_edge_distance=0.0,
        nearest_arc_length_m=0.0,
        fit_method='manual',
        fit_quality_mean_error_m=0.0,
        closure_gap_m=0.0,
        signed_area=0.0,
        point_count_input=0,
        point_count_filtered=0,
        point_count_samples=0,
    )
    work_band = WorkBandPlan(
        strategy='far_field',
        strategy_reason='test',
        config=WorkBandConfig(
            min_offset_m=2.0,
            preferred_offset_m=2.5,
            max_offset_m=3.0,
            obstacle_clearance_m=0.8,
            max_slope_deg=10.0,
            min_turn_radius_m=4.0,
        ),
        stations=(),
        kept_station_indices=(),
        candidates=(
            WorkBandCandidate(
                anchor_x=-1.0,
                anchor_y=0.0,
                anchor_z=0.0,
                target_x=-3.0,
                target_y=0.0,
                target_z=0.0,
                face_x=-1.0,
                face_y=0.0,
                anchor_arc_length_m=1.0,
                anchor_curvature=0.1,
                target_offset_m=2.0,
                approach_distance_m=3.0,
                heading_error_deg=5.0,
                nearest_obstacle_clearance_m=0.1,
                local_slope_deg=8.0,
                estimated_turn_radius_m=1.0,
                source='unsafe_close',
            ),
            WorkBandCandidate(
                anchor_x=1.0,
                anchor_y=0.0,
                anchor_z=0.0,
                target_x=-1.5,
                target_y=0.0,
                target_z=0.0,
                face_x=1.0,
                face_y=0.0,
                anchor_arc_length_m=2.0,
                anchor_curvature=0.05,
                target_offset_m=2.5,
                approach_distance_m=4.5,
                heading_error_deg=8.0,
                nearest_obstacle_clearance_m=3.0,
                local_slope_deg=1.0,
                estimated_turn_radius_m=9.0,
                source='safe_farther',
            ),
        ),
    )

    target, debug = evaluate_work_band(
        boundary=boundary,
        work_band=work_band,
        weights=CandidateWeights(
            path_weight=1.0,
            reference_weight=0.0,
            yaw_weight=0.1,
            distance_weight=0.2,
            safety_weight=8.0,
            curvature_weight=0.0,
        ),
    )

    assert debug['candidate_evaluation']['candidate_source'] == 'safe_farther'
    assert math.isclose(target.pose.position.x, -1.5, abs_tol=1e-6)


def test_material_target_planner_node_uses_pointcloud_extractor_branch() -> None:
    rclpy.init()
    node = MaterialTargetPlannerNode()
    try:
        request = ComputeMaterialTarget.Request()
        request.request_id = 'pointcloud-branch'
        request.current_pose = _pose(0.0, -5.0)
        request.material_reference_pose = _pose(0.0, 0.0)
        request.material_outline = []
        request.desired_standoff_m = 2.5
        request.material_profile_json = '{}'
        request.planner_constraints_json = json.dumps({'boundary_input': {'source': 'pointcloud'}})

        node._request_extracted_boundary = lambda _: (
            [
                Point(x=-1.0, y=-1.0, z=0.0),
                Point(x=1.0, y=-1.0, z=0.0),
                Point(x=1.0, y=1.0, z=0.0),
                Point(x=-1.0, y=1.0, z=0.0),
            ],
            {'status': 'ok', 'outline_point_count': 4},
        )

        response = node._handle_request(request, ComputeMaterialTarget.Response())
        debug = json.loads(response.debug_json)

        assert response.success is True
        assert debug['geometry_input']['source'] == 'pointcloud_boundary_extractor'
        assert debug['geometry_input']['extracted_outline_count'] == 4
        assert debug['pointcloud_boundary_extractor']['status'] == 'ok'
    finally:
        node.destroy_node()
        rclpy.shutdown()
