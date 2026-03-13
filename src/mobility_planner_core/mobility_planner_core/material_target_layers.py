from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from geometry_msgs.msg import Point, PoseStamped


DEFAULT_EDGE_NEAR_THRESHOLD_M = 3.0
DEFAULT_MAX_CANDIDATES = 48
DEFAULT_BOUNDARY_GRID_SIZE_M = 0.5
DEFAULT_BOUNDARY_SAMPLE_COUNT = 64
MIN_NORMAL_ALIGNMENT = 0.35


@dataclass(frozen=True)
class CandidateWeights:
    path_weight: float = 1.0
    reference_weight: float = 0.35
    yaw_weight: float = 0.2
    distance_weight: float = 0.25
    safety_weight: float = 0.8
    curvature_weight: float = 0.2


@dataclass(frozen=True)
class BoundaryFitConfig:
    grid_size_m: float = DEFAULT_BOUNDARY_GRID_SIZE_M
    min_points_per_cell: int = 1
    sample_count: int = DEFAULT_BOUNDARY_SAMPLE_COUNT


@dataclass(frozen=True)
class BoundaryCurveSample:
    x: float
    y: float
    z: float
    arc_length_m: float
    tangent_x: float
    tangent_y: float
    normal_x: float
    normal_y: float
    curvature: float
    source: str


@dataclass(frozen=True)
class MaterialBoundaryModel:
    frame_id: str
    centroid_x: float
    centroid_y: float
    centroid_z: float
    centroid_source: str
    current_x: float
    current_y: float
    current_z: float
    current_heading: float
    reference_x: float
    reference_y: float
    reference_z: float
    ordered_boundary_points: tuple[Point, ...]
    curve_samples: tuple[BoundaryCurveSample, ...]
    total_arc_length_m: float
    nearest_edge_point: Point | None
    nearest_edge_distance: float
    nearest_arc_length_m: float
    fit_method: str
    fit_quality_mean_error_m: float
    closure_gap_m: float
    signed_area: float
    point_count_input: int
    point_count_filtered: int
    point_count_samples: int


@dataclass(frozen=True)
class WorkBandCandidate:
    anchor_x: float
    anchor_y: float
    anchor_z: float
    target_x: float
    target_y: float
    target_z: float
    face_x: float
    face_y: float
    anchor_arc_length_m: float
    anchor_curvature: float
    target_offset_m: float
    approach_distance_m: float
    heading_error_deg: float
    nearest_obstacle_clearance_m: float
    local_slope_deg: float
    estimated_turn_radius_m: float
    source: str


@dataclass(frozen=True)
class CircularObstacle:
    x: float
    y: float
    radius_m: float


@dataclass(frozen=True)
class SlopeZone:
    x: float
    y: float
    radius_m: float
    slope_deg: float


@dataclass(frozen=True)
class WorkBandConfig:
    min_offset_m: float
    preferred_offset_m: float
    max_offset_m: float
    edge_near_threshold_m: float = DEFAULT_EDGE_NEAR_THRESHOLD_M
    max_candidates: int = DEFAULT_MAX_CANDIDATES
    min_approach_distance_m: float = 0.0
    max_approach_distance_m: float = 50.0
    max_heading_change_deg: float = 135.0
    max_boundary_curvature: float = 1.5
    min_turn_radius_m: float = 0.0
    obstacle_clearance_m: float = 0.0
    max_slope_deg: float = 90.0
    preferred_yaw_offset_deg: float = 0.0
    obstacles: tuple[CircularObstacle, ...] = ()
    slope_zones: tuple[SlopeZone, ...] = ()


@dataclass(frozen=True)
class WorkBandStation:
    sample: BoundaryCurveSample
    inner_x: float
    inner_y: float
    preferred_x: float
    preferred_y: float
    outer_x: float
    outer_y: float
    target_yaw: float
    approach_distance_m: float
    heading_error_deg: float
    nearest_obstacle_clearance_m: float
    local_slope_deg: float
    estimated_turn_radius_m: float
    feasible: bool
    feasibility_reasons: tuple[str, ...]


@dataclass(frozen=True)
class WorkBandPlan:
    strategy: str
    strategy_reason: str
    config: WorkBandConfig
    stations: tuple[WorkBandStation, ...]
    kept_station_indices: tuple[int, ...]
    candidates: tuple[WorkBandCandidate, ...]


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def quaternion_to_yaw(pose: PoseStamped) -> float:
    z = pose.pose.orientation.z
    w = pose.pose.orientation.w
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def point_mean(points: Iterable[Point]) -> tuple[float, float, float] | None:
    pts = tuple(points)
    if not pts:
        return None
    return (
        sum(point.x for point in pts) / len(pts),
        sum(point.y for point in pts) / len(pts),
        sum(point.z for point in pts) / len(pts),
    )


def normalize(x: float, y: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm < 1e-6:
        return (-1.0, 0.0)
    return (x / norm, y / norm)


def distance(a_x: float, a_y: float, b_x: float, b_y: float) -> float:
    return math.hypot(a_x - b_x, a_y - b_y)


def angle_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def build_target_pose(
    *,
    frame_id: str,
    target_x: float,
    target_y: float,
    target_z: float,
    face_x: float,
    face_y: float,
) -> tuple[PoseStamped, float]:
    yaw = math.atan2(face_y - target_y, face_x - target_x)
    qz, qw = yaw_to_quaternion(yaw)
    target = PoseStamped()
    target.header.frame_id = frame_id or 'map'
    target.pose.position.x = target_x
    target.pose.position.y = target_y
    target.pose.position.z = target_z
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw
    return target, yaw


def _deduplicate_points(points: tuple[Point, ...]) -> tuple[Point, ...]:
    seen: set[tuple[float, float, float]] = set()
    unique: list[Point] = []
    for point in points:
        key = (round(point.x, 6), round(point.y, 6), round(point.z, 6))
        if key in seen:
            continue
        seen.add(key)
        unique.append(Point(x=point.x, y=point.y, z=point.z))
    return tuple(unique)


def _grid_filter_points(points: tuple[Point, ...], config: BoundaryFitConfig) -> tuple[Point, ...]:
    if not points or config.grid_size_m <= 1e-6:
        return points
    buckets: dict[tuple[int, int], list[Point]] = {}
    for point in points:
        key = (
            math.floor(point.x / config.grid_size_m),
            math.floor(point.y / config.grid_size_m),
        )
        buckets.setdefault(key, []).append(point)
    filtered: list[Point] = []
    for cell_points in buckets.values():
        if len(cell_points) < config.min_points_per_cell:
            if config.min_points_per_cell > 1:
                continue
        mean_x = sum(point.x for point in cell_points) / len(cell_points)
        mean_y = sum(point.y for point in cell_points) / len(cell_points)
        mean_z = sum(point.z for point in cell_points) / len(cell_points)
        filtered.append(Point(x=mean_x, y=mean_y, z=mean_z))
    if not filtered:
        return points
    return tuple(filtered)


def _sort_points_polar(points: tuple[Point, ...], centroid_x: float, centroid_y: float) -> tuple[Point, ...]:
    return tuple(
        sorted(
            points,
            key=lambda point: (math.atan2(point.y - centroid_y, point.x - centroid_x), distance(point.x, point.y, centroid_x, centroid_y)),
        )
    )


def _signed_area(points: tuple[Point, ...]) -> float:
    if len(points) < 3:
        return 0.0
    area = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        area += (point.x * nxt.y) - (nxt.x * point.y)
    return 0.5 * area


def _polyline_length(points: tuple[Point, ...], closed: bool) -> tuple[list[float], float]:
    cumulative = [0.0]
    total = 0.0
    count = len(points)
    if count <= 1:
        return cumulative, total
    segment_count = count if closed else count - 1
    for index in range(segment_count):
        start = points[index]
        end = points[(index + 1) % count]
        total += distance(start.x, start.y, end.x, end.y)
        cumulative.append(total)
    return cumulative, total


def _interpolate_polyline(points: tuple[Point, ...], cumulative: list[float], s_value: float, closed: bool) -> Point:
    if len(points) == 1:
        point = points[0]
        return Point(x=point.x, y=point.y, z=point.z)
    total = cumulative[-1]
    if total <= 1e-6:
        point = points[0]
        return Point(x=point.x, y=point.y, z=point.z)
    if closed:
        s_value = s_value % total
    else:
        s_value = min(max(s_value, 0.0), total)
    for index in range(1, len(cumulative)):
        if s_value <= cumulative[index]:
            start = points[index - 1]
            end = points[index % len(points)] if closed else points[index]
            segment_length = max(cumulative[index] - cumulative[index - 1], 1e-6)
            ratio = (s_value - cumulative[index - 1]) / segment_length
            return Point(
                x=start.x + ((end.x - start.x) * ratio),
                y=start.y + ((end.y - start.y) * ratio),
                z=start.z + ((end.z - start.z) * ratio),
            )
    return Point(x=points[-1].x, y=points[-1].y, z=points[-1].z)


def _distance_point_to_segment(px: float, py: float, start: Point, end: Point) -> float:
    segment_x = end.x - start.x
    segment_y = end.y - start.y
    denom = (segment_x * segment_x) + (segment_y * segment_y)
    if denom <= 1e-9:
        return distance(px, py, start.x, start.y)
    ratio = ((px - start.x) * segment_x + (py - start.y) * segment_y) / denom
    ratio = min(max(ratio, 0.0), 1.0)
    proj_x = start.x + (segment_x * ratio)
    proj_y = start.y + (segment_y * ratio)
    return distance(px, py, proj_x, proj_y)


def _mean_fit_error(points: tuple[Point, ...], ordered_points: tuple[Point, ...], closed: bool) -> float:
    if len(ordered_points) < 2 or not points:
        return 0.0
    segment_count = len(ordered_points) if closed else len(ordered_points) - 1
    distances: list[float] = []
    for point in points:
        segment_distance = min(
            _distance_point_to_segment(point.x, point.y, ordered_points[index], ordered_points[(index + 1) % len(ordered_points)])
            for index in range(segment_count)
        )
        distances.append(segment_distance)
    return sum(distances) / len(distances)


def _closure_gap(points: tuple[Point, ...], closed: bool) -> float:
    if not closed or len(points) < 2:
        return 0.0
    return distance(points[0].x, points[0].y, points[-1].x, points[-1].y)


def _build_curve_samples(
    points: tuple[Point, ...],
    *,
    centroid_x: float,
    centroid_y: float,
    sample_count: int,
    closed: bool,
    fit_method: str,
) -> tuple[BoundaryCurveSample, ...]:
    if not points:
        return ()
    if len(points) == 1:
        point = points[0]
        normal_x, normal_y = normalize(point.x - centroid_x, point.y - centroid_y)
        return (
            BoundaryCurveSample(
                x=point.x,
                y=point.y,
                z=point.z,
                arc_length_m=0.0,
                tangent_x=0.0,
                tangent_y=0.0,
                normal_x=normal_x,
                normal_y=normal_y,
                curvature=0.0,
                source=fit_method,
            ),
        )

    cumulative, total_length = _polyline_length(points, closed=closed)
    effective_count = max(len(points), sample_count)
    if total_length <= 1e-6:
        return tuple(
            BoundaryCurveSample(
                x=point.x,
                y=point.y,
                z=point.z,
                arc_length_m=0.0,
                tangent_x=0.0,
                tangent_y=0.0,
                normal_x=normalize(point.x - centroid_x, point.y - centroid_y)[0],
                normal_y=normalize(point.x - centroid_x, point.y - centroid_y)[1],
                curvature=0.0,
                source=fit_method,
            )
            for point in points
        )

    step = total_length / effective_count if closed else total_length / max(effective_count - 1, 1)
    raw_samples = [
        _interpolate_polyline(points, cumulative, index * step, closed=closed)
        for index in range(effective_count)
    ]
    if not closed:
        raw_samples[-1] = Point(x=points[-1].x, y=points[-1].y, z=points[-1].z)

    samples: list[BoundaryCurveSample] = []
    for index, point in enumerate(raw_samples):
        prev_point = raw_samples[(index - 1) % len(raw_samples)] if closed else raw_samples[max(index - 1, 0)]
        next_point = raw_samples[(index + 1) % len(raw_samples)] if closed else raw_samples[min(index + 1, len(raw_samples) - 1)]
        tangent_x, tangent_y = normalize(next_point.x - prev_point.x, next_point.y - prev_point.y)
        radial_x, radial_y = normalize(point.x - centroid_x, point.y - centroid_y)
        raw_normal_x, raw_normal_y = -tangent_y, tangent_x
        alignment = abs((raw_normal_x * radial_x) + (raw_normal_y * radial_y))
        if alignment < MIN_NORMAL_ALIGNMENT:
            normal_x, normal_y = radial_x, radial_y
        else:
            if ((raw_normal_x * radial_x) + (raw_normal_y * radial_y)) < 0.0:
                raw_normal_x *= -1.0
                raw_normal_y *= -1.0
            normal_x, normal_y = normalize(raw_normal_x, raw_normal_y)

        prev_tangent_x, prev_tangent_y = normalize(point.x - prev_point.x, point.y - prev_point.y)
        next_tangent_x, next_tangent_y = normalize(next_point.x - point.x, next_point.y - point.y)
        tangent_turn = angle_diff(math.atan2(next_tangent_y, next_tangent_x), math.atan2(prev_tangent_y, prev_tangent_x))
        local_step = max(distance(prev_point.x, prev_point.y, next_point.x, next_point.y), 1e-6)
        curvature = tangent_turn / local_step
        samples.append(
            BoundaryCurveSample(
                x=point.x,
                y=point.y,
                z=point.z,
                arc_length_m=index * step,
                tangent_x=tangent_x,
                tangent_y=tangent_y,
                normal_x=normal_x,
                normal_y=normal_y,
                curvature=curvature,
                source=fit_method,
            )
        )
    return tuple(samples)


def _nearest_sample(x: float, y: float, samples: tuple[BoundaryCurveSample, ...]) -> tuple[BoundaryCurveSample | None, float]:
    if not samples:
        return None, math.inf
    sample = min(samples, key=lambda item: distance(x, y, item.x, item.y))
    return sample, distance(x, y, sample.x, sample.y)


def _resample_anchor_points(points: tuple[Point, ...], max_candidates: int) -> tuple[Point, ...]:
    if len(points) <= max_candidates:
        return points
    stride = max(1, len(points) // max_candidates)
    sampled = list(points[::stride])
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return tuple(sampled[:max_candidates])


def fit_material_boundary(
    *,
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
    material_outline: Iterable[Point],
    fit_config: BoundaryFitConfig | None = None,
) -> MaterialBoundaryModel:
    config = fit_config or BoundaryFitConfig()
    input_points = tuple(material_outline)
    unique_points = _deduplicate_points(input_points)
    centroid = point_mean(unique_points)
    if centroid is not None:
        centroid_x, centroid_y, centroid_z = centroid
        centroid_source = 'outline_mean'
    else:
        centroid_x = material_reference_pose.pose.position.x
        centroid_y = material_reference_pose.pose.position.y
        centroid_z = material_reference_pose.pose.position.z
        centroid_source = 'material_reference_pose'

    filtered_points = _grid_filter_points(unique_points, config)
    if not filtered_points:
        filtered_points = unique_points

    current_x = current_pose.pose.position.x
    current_y = current_pose.pose.position.y
    current_z = current_pose.pose.position.z if current_pose.header.frame_id else centroid_z
    frame_id = material_reference_pose.header.frame_id or current_pose.header.frame_id or 'map'

    if len(filtered_points) >= 3:
        ordered_points = _sort_points_polar(filtered_points, centroid_x, centroid_y)
        signed_area = _signed_area(ordered_points)
        closed = abs(signed_area) > 1e-6
        fit_method = 'piecewise_cubic_boundary' if closed else 'degenerate_line_boundary'
    else:
        ordered_points = tuple(
            sorted(filtered_points, key=lambda point: (point.x, point.y, point.z))
        )
        signed_area = 0.0
        closed = False
        fit_method = 'degenerate_line_boundary'

    curve_samples = _build_curve_samples(
        ordered_points,
        centroid_x=centroid_x,
        centroid_y=centroid_y,
        sample_count=config.sample_count,
        closed=closed,
        fit_method=fit_method,
    )
    nearest_sample, nearest_edge_distance = _nearest_sample(current_x, current_y, curve_samples)
    nearest_edge_point = None
    nearest_arc_length_m = 0.0
    if nearest_sample is not None:
        nearest_edge_point = Point(x=nearest_sample.x, y=nearest_sample.y, z=nearest_sample.z)
        nearest_arc_length_m = nearest_sample.arc_length_m
    fit_quality_mean_error_m = _mean_fit_error(unique_points, ordered_points, closed=closed)
    total_arc_length = _polyline_length(ordered_points, closed=closed)[1]
    return MaterialBoundaryModel(
        frame_id=frame_id,
        centroid_x=centroid_x,
        centroid_y=centroid_y,
        centroid_z=centroid_z,
        centroid_source=centroid_source,
        current_x=current_x,
        current_y=current_y,
        current_z=current_z,
        current_heading=quaternion_to_yaw(current_pose),
        reference_x=material_reference_pose.pose.position.x,
        reference_y=material_reference_pose.pose.position.y,
        reference_z=material_reference_pose.pose.position.z,
        ordered_boundary_points=ordered_points,
        curve_samples=curve_samples,
        total_arc_length_m=total_arc_length,
        nearest_edge_point=nearest_edge_point,
        nearest_edge_distance=nearest_edge_distance,
        nearest_arc_length_m=nearest_arc_length_m,
        fit_method=fit_method,
        fit_quality_mean_error_m=fit_quality_mean_error_m,
        closure_gap_m=_closure_gap(ordered_points, closed=closed),
        signed_area=signed_area,
        point_count_input=len(input_points),
        point_count_filtered=len(filtered_points),
        point_count_samples=len(curve_samples),
    )


def select_strategy(
    *,
    strategy_hint: str,
    nearest_edge_distance: float,
    edge_near_threshold_m: float,
    outline_available: bool,
) -> tuple[str, str]:
    if strategy_hint in {'edge_near', 'near_edge', 'edge'}:
        return 'edge_near', 'forced_by_hint'
    if strategy_hint in {'far_field', 'far'}:
        return 'far_field', 'forced_by_hint'
    if not outline_available:
        return 'far_field', 'outline_unavailable'
    if nearest_edge_distance <= edge_near_threshold_m:
        return 'edge_near', 'current_pose_close_to_material_edge'
    return 'far_field', 'current_pose_far_from_material_edge'


def _sample_to_candidate(sample: BoundaryCurveSample, boundary: MaterialBoundaryModel, standoff_m: float, source: str) -> WorkBandCandidate:
    return WorkBandCandidate(
        anchor_x=sample.x,
        anchor_y=sample.y,
        anchor_z=sample.z,
        target_x=sample.x + (sample.normal_x * standoff_m),
        target_y=sample.y + (sample.normal_y * standoff_m),
        target_z=boundary.current_z,
        face_x=sample.x,
        face_y=sample.y,
        anchor_arc_length_m=sample.arc_length_m,
        anchor_curvature=sample.curvature,
        target_offset_m=standoff_m,
        approach_distance_m=distance(boundary.current_x, boundary.current_y, sample.x + (sample.normal_x * standoff_m), sample.y + (sample.normal_y * standoff_m)),
        heading_error_deg=0.0,
        nearest_obstacle_clearance_m=math.inf,
        local_slope_deg=0.0,
        estimated_turn_radius_m=math.inf,
        source=source,
    )


def _station_to_candidate(station: WorkBandStation, boundary: MaterialBoundaryModel, source: str) -> WorkBandCandidate:
    return WorkBandCandidate(
        anchor_x=station.sample.x,
        anchor_y=station.sample.y,
        anchor_z=station.sample.z,
        target_x=station.preferred_x,
        target_y=station.preferred_y,
        target_z=boundary.current_z,
        face_x=station.sample.x,
        face_y=station.sample.y,
        anchor_arc_length_m=station.sample.arc_length_m,
        anchor_curvature=station.sample.curvature,
        target_offset_m=distance(station.sample.x, station.sample.y, station.preferred_x, station.preferred_y),
        approach_distance_m=station.approach_distance_m,
        heading_error_deg=station.heading_error_deg,
        nearest_obstacle_clearance_m=station.nearest_obstacle_clearance_m,
        local_slope_deg=station.local_slope_deg,
        estimated_turn_radius_m=station.estimated_turn_radius_m,
        source=source,
    )


def _point_to_candidate(
    anchor: Point,
    *,
    normal_x: float,
    normal_y: float,
    boundary: MaterialBoundaryModel,
    standoff_m: float,
    source: str,
) -> WorkBandCandidate:
    return WorkBandCandidate(
        anchor_x=anchor.x,
        anchor_y=anchor.y,
        anchor_z=anchor.z,
        target_x=anchor.x + (normal_x * standoff_m),
        target_y=anchor.y + (normal_y * standoff_m),
        target_z=boundary.current_z,
        face_x=anchor.x,
        face_y=anchor.y,
        anchor_arc_length_m=0.0,
        anchor_curvature=0.0,
        target_offset_m=standoff_m,
        approach_distance_m=distance(boundary.current_x, boundary.current_y, anchor.x + (normal_x * standoff_m), anchor.y + (normal_y * standoff_m)),
        heading_error_deg=0.0,
        nearest_obstacle_clearance_m=math.inf,
        local_slope_deg=0.0,
        estimated_turn_radius_m=math.inf,
        source=source,
    )


def _resolve_target_yaw(sample: BoundaryCurveSample, yaw_offset_deg: float) -> float:
    inward_normal_yaw = math.atan2(-sample.normal_y, -sample.normal_x)
    return inward_normal_yaw + math.radians(yaw_offset_deg)


def _heading_error_deg(current_heading: float, target_heading: float) -> float:
    return math.degrees(angle_diff(current_heading, target_heading))


def _nearest_obstacle_clearance(x: float, y: float, obstacles: tuple[CircularObstacle, ...]) -> float:
    if not obstacles:
        return math.inf
    return min(distance(x, y, obstacle.x, obstacle.y) - obstacle.radius_m for obstacle in obstacles)


def _local_slope_deg(x: float, y: float, slope_zones: tuple[SlopeZone, ...]) -> float:
    if not slope_zones:
        return 0.0
    slope_deg = 0.0
    for zone in slope_zones:
        if distance(x, y, zone.x, zone.y) <= zone.radius_m:
            slope_deg = max(slope_deg, zone.slope_deg)
    return slope_deg


def _estimate_turn_radius(
    *,
    current_x: float,
    current_y: float,
    current_heading: float,
    target_x: float,
    target_y: float,
    target_yaw: float,
) -> float:
    chord = distance(current_x, current_y, target_x, target_y)
    if chord <= 1e-6:
        return math.inf
    heading_to_target = math.atan2(target_y - current_y, target_x - current_x)
    entry_delta = angle_diff(current_heading, heading_to_target)
    exit_delta = angle_diff(heading_to_target, target_yaw)

    def radius_for_delta(delta: float) -> float:
        if delta <= 1e-6:
            return math.inf
        return chord / max(2.0 * math.sin(delta * 0.5), 1e-6)

    return min(radius_for_delta(entry_delta), radius_for_delta(exit_delta))


def _build_work_band_stations(
    *,
    boundary: MaterialBoundaryModel,
    config: WorkBandConfig,
) -> tuple[WorkBandStation, ...]:
    if not boundary.curve_samples:
        return ()
    stations: list[WorkBandStation] = []
    for sample in boundary.curve_samples:
        inner_x = sample.x + (sample.normal_x * config.min_offset_m)
        inner_y = sample.y + (sample.normal_y * config.min_offset_m)
        preferred_x = sample.x + (sample.normal_x * config.preferred_offset_m)
        preferred_y = sample.y + (sample.normal_y * config.preferred_offset_m)
        outer_x = sample.x + (sample.normal_x * config.max_offset_m)
        outer_y = sample.y + (sample.normal_y * config.max_offset_m)
        target_yaw = _resolve_target_yaw(sample, config.preferred_yaw_offset_deg)
        approach_distance_m = distance(boundary.current_x, boundary.current_y, preferred_x, preferred_y)
        heading_error_deg = _heading_error_deg(boundary.current_heading, target_yaw)
        nearest_obstacle_clearance_m = _nearest_obstacle_clearance(preferred_x, preferred_y, config.obstacles)
        local_slope_deg = _local_slope_deg(preferred_x, preferred_y, config.slope_zones)
        estimated_turn_radius_m = _estimate_turn_radius(
            current_x=boundary.current_x,
            current_y=boundary.current_y,
            current_heading=boundary.current_heading,
            target_x=preferred_x,
            target_y=preferred_y,
            target_yaw=target_yaw,
        )
        reasons: list[str] = []
        if approach_distance_m < config.min_approach_distance_m:
            reasons.append('approach_too_short')
        if approach_distance_m > config.max_approach_distance_m:
            reasons.append('approach_too_long')
        if heading_error_deg > config.max_heading_change_deg:
            reasons.append('heading_change_too_large')
        if sample.curvature > config.max_boundary_curvature:
            reasons.append('boundary_curvature_too_large')
        if config.min_turn_radius_m > 0.0 and estimated_turn_radius_m < config.min_turn_radius_m:
            reasons.append('turn_radius_too_small')
        if nearest_obstacle_clearance_m < config.obstacle_clearance_m:
            reasons.append('obstacle_clearance_too_small')
        if local_slope_deg > config.max_slope_deg:
            reasons.append('local_slope_too_large')
        stations.append(
            WorkBandStation(
                sample=sample,
                inner_x=inner_x,
                inner_y=inner_y,
                preferred_x=preferred_x,
                preferred_y=preferred_y,
                outer_x=outer_x,
                outer_y=outer_y,
                target_yaw=target_yaw,
                approach_distance_m=approach_distance_m,
                heading_error_deg=heading_error_deg,
                nearest_obstacle_clearance_m=nearest_obstacle_clearance_m,
                local_slope_deg=local_slope_deg,
                estimated_turn_radius_m=estimated_turn_radius_m,
                feasible=not reasons,
                feasibility_reasons=tuple(reasons),
            )
        )
    return tuple(stations)


def _largest_feasible_segment(stations: tuple[WorkBandStation, ...], closed: bool) -> tuple[int, ...]:
    if not stations:
        return ()
    feasible_indices = tuple(index for index, station in enumerate(stations) if station.feasible)
    if len(feasible_indices) == len(stations):
        return tuple(range(len(stations)))
    if not feasible_indices:
        return ()
    if not closed:
        best: list[int] = []
        current: list[int] = []
        for index, station in enumerate(stations):
            if station.feasible:
                current.append(index)
                if len(current) > len(best):
                    best = current.copy()
            else:
                current.clear()
        return tuple(best)

    doubled = list(stations) + list(stations)
    best_start = -1
    best_len = 0
    current_start = None
    current_len = 0
    for index, station in enumerate(doubled):
        if station.feasible:
            if current_start is None:
                current_start = index
                current_len = 1
            else:
                current_len += 1
            current_len = min(current_len, len(stations))
            if current_len > best_len:
                best_start = current_start
                best_len = current_len
        else:
            current_start = None
            current_len = 0
    if best_len == 0:
        return ()
    start = best_start % len(stations)
    return tuple((start + offset) % len(stations) for offset in range(best_len))


def _resample_station_indices(indices: tuple[int, ...], max_candidates: int) -> tuple[int, ...]:
    if len(indices) <= max_candidates:
        return indices
    stride = max(1, len(indices) // max_candidates)
    sampled = list(indices[::stride])
    if sampled[-1] != indices[-1]:
        sampled.append(indices[-1])
    return tuple(sampled[:max_candidates])


def generate_work_band(
    *,
    boundary: MaterialBoundaryModel,
    config: WorkBandConfig,
    strategy_hint: str = 'auto',
) -> WorkBandPlan:
    strategy, strategy_reason = select_strategy(
        strategy_hint=strategy_hint.strip().lower(),
        nearest_edge_distance=boundary.nearest_edge_distance,
        edge_near_threshold_m=config.edge_near_threshold_m,
        outline_available=bool(boundary.curve_samples),
    )
    degenerate_boundary = boundary.fit_method == 'degenerate_line_boundary'
    current_outward_x, current_outward_y = normalize(
        boundary.current_x - boundary.centroid_x,
        boundary.current_y - boundary.centroid_y,
    )
    stations = _build_work_band_stations(boundary=boundary, config=config)
    kept_station_indices = _largest_feasible_segment(
        stations,
        closed=(boundary.fit_method == 'piecewise_cubic_boundary'),
    )
    kept_stations = tuple(stations[index] for index in kept_station_indices)

    if strategy == 'edge_near':
        if kept_stations and not degenerate_boundary:
            station = min(
                kept_stations,
                key=lambda item: distance(boundary.current_x, boundary.current_y, item.preferred_x, item.preferred_y),
            )
            candidates = (_station_to_candidate(station, boundary, 'edge_near_work_band'),)
        else:
            anchor = boundary.nearest_edge_point or Point(x=boundary.centroid_x, y=boundary.centroid_y, z=boundary.centroid_z)
            candidates = (
                _point_to_candidate(
                    anchor,
                    normal_x=current_outward_x,
                    normal_y=current_outward_y,
                    boundary=boundary,
                    standoff_m=config.preferred_offset_m,
                    source='edge_near_centroid_fallback',
                ),
            )
        return WorkBandPlan(
            strategy=strategy,
            strategy_reason=strategy_reason,
            config=config,
            stations=stations,
            kept_station_indices=kept_station_indices,
            candidates=candidates,
        )

    candidates: list[WorkBandCandidate] = []
    if kept_stations and not degenerate_boundary:
        approach_station = max(
            kept_stations,
            key=lambda station: (station.sample.normal_x * current_outward_x) + (station.sample.normal_y * current_outward_y),
        )
        candidates.append(_station_to_candidate(approach_station, boundary, 'current_approach'))
        for index in _resample_station_indices(kept_station_indices, config.max_candidates):
            candidates.append(_station_to_candidate(stations[index], boundary, 'work_band_strip'))
    else:
        candidates.append(
            _point_to_candidate(
                Point(x=boundary.centroid_x, y=boundary.centroid_y, z=boundary.centroid_z),
                normal_x=current_outward_x,
                normal_y=current_outward_y,
                boundary=boundary,
                standoff_m=config.preferred_offset_m,
                source='current_approach',
            )
        )
        for anchor_point in _resample_anchor_points(boundary.ordered_boundary_points, config.max_candidates):
            candidates.append(
                _point_to_candidate(
                    anchor_point,
                    normal_x=current_outward_x,
                    normal_y=current_outward_y,
                    boundary=boundary,
                    standoff_m=config.preferred_offset_m,
                    source='outline_band',
                )
            )
    if not candidates:
        candidates.append(
            WorkBandCandidate(
                anchor_x=boundary.reference_x,
                anchor_y=boundary.reference_y,
                anchor_z=boundary.reference_z,
                target_x=boundary.reference_x + (current_outward_x * config.preferred_offset_m),
                target_y=boundary.reference_y + (current_outward_y * config.preferred_offset_m),
                target_z=boundary.current_z,
                face_x=boundary.reference_x,
                face_y=boundary.reference_y,
                anchor_arc_length_m=0.0,
                anchor_curvature=0.0,
                target_offset_m=config.preferred_offset_m,
                approach_distance_m=distance(
                    boundary.current_x,
                    boundary.current_y,
                    boundary.reference_x + (current_outward_x * config.preferred_offset_m),
                    boundary.reference_y + (current_outward_y * config.preferred_offset_m),
                ),
                heading_error_deg=0.0,
                nearest_obstacle_clearance_m=math.inf,
                local_slope_deg=0.0,
                estimated_turn_radius_m=math.inf,
                source='material_reference_pose',
            )
        )

    unique_candidates: list[WorkBandCandidate] = []
    seen: set[tuple[float, float, str]] = set()
    for candidate in candidates:
        key = (round(candidate.anchor_x, 3), round(candidate.anchor_y, 3), candidate.source)
        if key in seen:
            continue
        seen.add(key)
        unique_candidates.append(candidate)
    return WorkBandPlan(
        strategy=strategy,
        strategy_reason=strategy_reason,
        config=config,
        stations=stations,
        kept_station_indices=kept_station_indices,
        candidates=tuple(unique_candidates),
    )


def evaluate_work_band(
    *,
    boundary: MaterialBoundaryModel,
    work_band: WorkBandPlan,
    weights: CandidateWeights,
) -> tuple[PoseStamped, dict]:
    best_target = None
    best_debug = None
    best_cost = math.inf
    for candidate in work_band.candidates:
        target, yaw = build_target_pose(
            frame_id=boundary.frame_id,
            target_x=candidate.target_x,
            target_y=candidate.target_y,
            target_z=candidate.target_z,
            face_x=candidate.face_x,
            face_y=candidate.face_y,
        )
        path_cost = candidate.approach_distance_m
        reference_cost = distance(boundary.reference_x, boundary.reference_y, candidate.anchor_x, candidate.anchor_y)
        heading_cost = candidate.heading_error_deg / 180.0
        distance_cost = abs(candidate.target_offset_m - work_band.config.preferred_offset_m) / max(work_band.config.preferred_offset_m, 1e-6)
        obstacle_cost = 0.0
        if math.isfinite(candidate.nearest_obstacle_clearance_m):
            obstacle_denom = max(work_band.config.obstacle_clearance_m, 1.0)
            obstacle_cost = 1.0 / max(candidate.nearest_obstacle_clearance_m + obstacle_denom, 1e-6)
        slope_cost = 0.0
        if work_band.config.max_slope_deg > 0.0:
            slope_cost = candidate.local_slope_deg / work_band.config.max_slope_deg
        turn_radius_cost = 0.0
        if work_band.config.min_turn_radius_m > 0.0 and math.isfinite(candidate.estimated_turn_radius_m):
            turn_radius_cost = max(0.0, work_band.config.min_turn_radius_m - candidate.estimated_turn_radius_m) / work_band.config.min_turn_radius_m
        safety_cost = obstacle_cost + slope_cost + turn_radius_cost
        curvature_cost = candidate.anchor_curvature
        total_cost = (
            (weights.path_weight * path_cost)
            + (weights.reference_weight * reference_cost)
            + (weights.distance_weight * distance_cost)
            + (weights.safety_weight * safety_cost)
            + (weights.curvature_weight * curvature_cost)
            + (weights.yaw_weight * heading_cost)
        )
        if total_cost >= best_cost:
            continue
        best_cost = total_cost
        best_target = target
        best_debug = {
            'anchor': {'x': candidate.anchor_x, 'y': candidate.anchor_y, 'z': candidate.anchor_z},
            'candidate_source': candidate.source,
            'anchor_arc_length_m': candidate.anchor_arc_length_m,
            'anchor_curvature': candidate.anchor_curvature,
            'target_offset_m': candidate.target_offset_m,
            'approach_distance_m': candidate.approach_distance_m,
            'heading_error_deg': candidate.heading_error_deg,
            'nearest_obstacle_clearance_m': candidate.nearest_obstacle_clearance_m,
            'local_slope_deg': candidate.local_slope_deg,
            'estimated_turn_radius_m': candidate.estimated_turn_radius_m,
            'yaw_rad': yaw,
            'selected_cost': total_cost,
            'cost_breakdown': {
                'path_cost': path_cost,
                'reference_cost': reference_cost,
                'distance_cost': distance_cost,
                'safety_cost': safety_cost,
                'curvature_cost': curvature_cost,
                'heading_cost': heading_cost,
                'obstacle_cost': obstacle_cost,
                'slope_cost': slope_cost,
                'turn_radius_cost': turn_radius_cost,
                'path_weight': weights.path_weight,
                'reference_weight': weights.reference_weight,
                'distance_weight': weights.distance_weight,
                'safety_weight': weights.safety_weight,
                'curvature_weight': weights.curvature_weight,
                'yaw_weight': weights.yaw_weight,
            },
        }
    assert best_target is not None
    assert best_debug is not None
    debug = {
        'boundary_fit': {
            'centroid': {
                'x': boundary.centroid_x,
                'y': boundary.centroid_y,
                'z': boundary.centroid_z,
            },
            'centroid_source': boundary.centroid_source,
            'fit_method': boundary.fit_method,
            'curve_length_m': boundary.total_arc_length_m,
            'nearest_edge_distance_m': boundary.nearest_edge_distance,
            'nearest_arc_length_m': boundary.nearest_arc_length_m,
            'closure_gap_m': boundary.closure_gap_m,
            'mean_fit_error_m': boundary.fit_quality_mean_error_m,
            'signed_area': boundary.signed_area,
            'point_count_input': boundary.point_count_input,
            'point_count_filtered': boundary.point_count_filtered,
            'point_count_samples': boundary.point_count_samples,
        },
        'work_band': {
            'strategy': work_band.strategy,
            'strategy_reason': work_band.strategy_reason,
            'min_offset_m': work_band.config.min_offset_m,
            'preferred_offset_m': work_band.config.preferred_offset_m,
            'max_offset_m': work_band.config.max_offset_m,
            'edge_near_threshold_m': work_band.config.edge_near_threshold_m,
            'station_count': len(work_band.stations),
            'kept_station_count': len(work_band.kept_station_indices),
            'candidate_count': len(work_band.candidates),
            'min_turn_radius_m': work_band.config.min_turn_radius_m,
            'obstacle_clearance_m': work_band.config.obstacle_clearance_m,
            'max_slope_deg': work_band.config.max_slope_deg,
        },
        'candidate_evaluation': best_debug,
    }
    return best_target, debug
