from __future__ import annotations

import math
from typing import Iterable

from geometry_msgs.msg import Point, PoseStamped


DEFAULT_SCATTER_ANGULAR_BINS = 72
DEFAULT_MIN_BOUNDARY_RADIUS_M = 0.15


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
            return Point(
                x=float(raw.get('x', 0.0)),
                y=float(raw.get('y', 0.0)),
                z=float(raw.get('z', 0.0)),
            )
        except (TypeError, ValueError):
            return None
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        try:
            return Point(
                x=float(raw[0]),
                y=float(raw[1]),
                z=float(raw[2]) if len(raw) >= 3 else 0.0,
            )
        except (TypeError, ValueError):
            return None
    return None


def _parse_points(raw: object) -> tuple[Point, ...]:
    if not isinstance(raw, list):
        return ()
    points: list[Point] = []
    for item in raw:
        point = _point_from_object(item)
        if point is not None:
            points.append(point)
    return tuple(points)


def _parse_line_strips(raw: object) -> tuple[Point, ...]:
    if not isinstance(raw, list):
        return ()
    flattened: list[Point] = []
    for strip in raw:
        if isinstance(strip, list):
            flattened.extend(_parse_points(strip))
    return tuple(flattened)


def _resolve_named_or_point(raw: object, *, current_pose: PoseStamped, material_reference_pose: PoseStamped) -> tuple[float, float, str]:
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


def _limit_points(points: tuple[Point, ...], max_points: int) -> tuple[Point, ...]:
    if max_points <= 0 or len(points) <= max_points:
        return points
    stride = max(1, len(points) // max_points)
    sampled = list(points[::stride])
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return tuple(sampled[:max_points])


def _filter_points_roi(points: tuple[Point, ...], *, center_x: float, center_y: float, radius_m: float) -> tuple[Point, ...]:
    if radius_m <= 0.0:
        return points
    kept = tuple(point for point in points if math.hypot(point.x - center_x, point.y - center_y) <= radius_m)
    return kept or points


def _extract_outline_from_scatter(
    scatter_points: tuple[Point, ...],
    *,
    center_x: float,
    center_y: float,
    angular_bins: int,
    min_boundary_radius_m: float,
) -> tuple[Point, ...]:
    if not scatter_points:
        return ()
    bins: dict[int, tuple[float, Point]] = {}
    for point in scatter_points:
        radial_distance = math.hypot(point.x - center_x, point.y - center_y)
        if radial_distance < min_boundary_radius_m:
            continue
        angle = math.atan2(point.y - center_y, point.x - center_x)
        index = int(((angle + math.pi) / (2.0 * math.pi)) * angular_bins) % angular_bins
        current = bins.get(index)
        if current is None or radial_distance > current[0]:
            bins[index] = (radial_distance, Point(x=point.x, y=point.y, z=point.z))
    if len(bins) >= 3:
        return tuple(bins[index][1] for index in sorted(bins.keys()))
    return scatter_points


def extract_outline_from_scatter_points(
    scatter_points: Iterable[Point],
    *,
    center_x: float,
    center_y: float,
    roi_center_x: float,
    roi_center_y: float,
    roi_radius_m: float,
    angular_bins: int,
    min_boundary_radius_m: float,
    max_input_points: int,
) -> tuple[tuple[Point, ...], dict]:
    raw_points = tuple(Point(x=point.x, y=point.y, z=point.z) for point in scatter_points)
    roi_points = _filter_points_roi(raw_points, center_x=roi_center_x, center_y=roi_center_y, radius_m=roi_radius_m)
    limited_points = _limit_points(roi_points, max_input_points)
    outline = _extract_outline_from_scatter(
        limited_points,
        center_x=center_x,
        center_y=center_y,
        angular_bins=angular_bins,
        min_boundary_radius_m=min_boundary_radius_m,
    )
    debug = {
        'scatter_point_count': len(raw_points),
        'roi_filtered_count': len(roi_points),
        'outline_point_count': len(outline),
        'roi_radius_m': roi_radius_m,
        'angular_bins': angular_bins,
        'min_boundary_radius_m': min_boundary_radius_m,
        'max_input_points': max_input_points,
    }
    return tuple(outline), debug


def resolve_material_outline(
    *,
    current_pose: PoseStamped,
    material_reference_pose: PoseStamped,
    material_outline: Iterable[Point],
    profile: dict,
    constraints: dict,
) -> tuple[tuple[Point, ...], dict]:
    input_config = _dict_section(profile, ('boundary_input', 'geometry_input'))
    input_constraints = _dict_section(constraints, ('boundary_input', 'geometry_input'))

    request_outline = tuple(Point(x=point.x, y=point.y, z=point.z) for point in material_outline)
    json_outline = _parse_points(input_constraints.get('outline_points', input_config.get('outline_points', [])))
    line_strip_points = _parse_line_strips(input_constraints.get('line_strips', input_config.get('line_strips', [])))
    scatter_points = _parse_points(
        input_constraints.get(
            'scatter_points',
            input_config.get('scatter_points', input_config.get('raw_points', [])),
        )
    )

    preferred_source = str(
        constraints.get(
            'geometry_source',
            input_constraints.get(
                'source',
                profile.get('geometry_source', input_config.get('source', 'auto')),
            ),
        )
    ).strip().lower()
    roi_radius_m = float(input_constraints.get('roi_radius_m', input_config.get('roi_radius_m', 0.0)))
    roi_center_x, roi_center_y, roi_center_source = _resolve_named_or_point(
        input_constraints.get('roi_center', input_config.get('roi_center', 'material_reference_pose')),
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
    )
    angular_bins = max(
        8,
        int(input_constraints.get('angular_bins', input_config.get('angular_bins', DEFAULT_SCATTER_ANGULAR_BINS))),
    )
    min_boundary_radius_m = max(
        0.0,
        float(
            input_constraints.get(
                'min_boundary_radius_m',
                input_config.get('min_boundary_radius_m', DEFAULT_MIN_BOUNDARY_RADIUS_M),
            )
        ),
    )
    max_input_points = max(0, int(input_constraints.get('max_input_points', input_config.get('max_input_points', 0))))
    centroid_hint_x, centroid_hint_y, centroid_hint_source = _resolve_named_or_point(
        input_constraints.get('centroid_hint', input_config.get('centroid_hint', 'material_reference_pose')),
        current_pose=current_pose,
        material_reference_pose=material_reference_pose,
    )

    selected_source = 'request_outline'
    selected_points = request_outline
    boundary_mode = 'direct_outline'

    if preferred_source in {'outline_points', 'json_outline'} and json_outline:
        selected_source = 'outline_points'
        selected_points = json_outline
    elif preferred_source in {'line_strips', 'line_strip'} and line_strip_points:
        selected_source = 'line_strips'
        selected_points = line_strip_points
    elif preferred_source in {'scatter_points', 'raw_points'} and scatter_points:
        selected_source = 'scatter_points'
        selected_points = scatter_points
        boundary_mode = 'scatter_boundary'
    elif preferred_source == 'auto':
        if request_outline:
            selected_source = 'request_outline'
            selected_points = request_outline
        elif json_outline:
            selected_source = 'outline_points'
            selected_points = json_outline
        elif line_strip_points:
            selected_source = 'line_strips'
            selected_points = line_strip_points
        elif scatter_points:
            selected_source = 'scatter_points'
            selected_points = scatter_points
            boundary_mode = 'scatter_boundary'

    raw_selected_count = len(selected_points)
    if boundary_mode == 'scatter_boundary':
        outline, scatter_debug = extract_outline_from_scatter_points(
            selected_points,
            center_x=centroid_hint_x,
            center_y=centroid_hint_y,
            roi_center_x=roi_center_x,
            roi_center_y=roi_center_y,
            roi_radius_m=roi_radius_m,
            angular_bins=angular_bins,
            min_boundary_radius_m=min_boundary_radius_m,
            max_input_points=max_input_points,
        )
        roi_filtered_count = scatter_debug['roi_filtered_count']
    else:
        roi_points = _filter_points_roi(selected_points, center_x=roi_center_x, center_y=roi_center_y, radius_m=roi_radius_m)
        limited_points = _limit_points(roi_points, max_input_points)
        outline = limited_points
        roi_filtered_count = len(roi_points)

    if not outline:
        outline = request_outline
        selected_source = 'request_outline_fallback'
        boundary_mode = 'direct_outline'

    debug = {
        'source': selected_source,
        'boundary_mode': boundary_mode,
        'preferred_source': preferred_source,
        'request_outline_count': len(request_outline),
        'json_outline_count': len(json_outline),
        'line_strip_point_count': len(line_strip_points),
        'scatter_point_count': len(scatter_points),
        'selected_input_count': raw_selected_count,
        'roi_filtered_count': roi_filtered_count,
        'outline_point_count': len(outline),
        'roi_radius_m': roi_radius_m,
        'roi_center_source': roi_center_source,
        'angular_bins': angular_bins,
        'min_boundary_radius_m': min_boundary_radius_m,
        'max_input_points': max_input_points,
        'centroid_hint_source': centroid_hint_source,
    }
    return tuple(outline), debug
