from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True)
class ViewTransform:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    width: float
    height: float
    padding: float
    scale: float
    x_offset: float
    y_offset: float


def resolve_workspace_root(start: Path) -> Path | None:
    markers = (
        Path('config/plc/mock_sequence.yaml'),
        Path('src/mission_bringup/launch/phase2_real.launch.py'),
    )
    for candidate in (start, *start.parents):
        if all((candidate / marker).exists() for marker in markers):
            return candidate
    return None


def parse_outline_text(text: str) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = [part.strip() for part in line.replace(';', ',').split(',') if part.strip()]
        if len(parts) < 2:
            raise ValueError(f'Invalid outline point: {raw_line!r}')
        points.append((float(parts[0]), float(parts[1])))
    return points


def normalize_json_text(raw: str) -> str:
    raw = raw.strip()
    if not raw:
        return '{}'
    return json.dumps(json.loads(raw), ensure_ascii=True, separators=(',', ':'))


def load_numeric_series_csv(path: Path) -> list[float]:
    text = path.read_text(encoding='utf-8').strip()
    if not text:
        return []
    values: list[float] = []
    for token in text.replace('\n', ',').split(','):
        token = token.strip()
        if token:
            values.append(float(token))
    return values


def load_xyz_csv(path: Path) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    with path.open('r', encoding='utf-8', newline='') as handle:
        reader = csv.reader(handle)
        for row in reader:
            values = [cell.strip() for cell in row if cell.strip()]
            if len(values) < 3:
                continue
            points.append((float(values[0]), float(values[1]), float(values[2])))
    return points


def downsample_points(points: Sequence[tuple[float, float]], max_points: int = 400) -> list[tuple[float, float]]:
    if len(points) <= max_points:
        return list(points)
    step = max(1, len(points) // max_points)
    sampled = [points[index] for index in range(0, len(points), step)]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def compute_bounds(point_sets: Iterable[Sequence[tuple[float, float]]]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for point_set in point_sets:
        for x_value, y_value in point_set:
            xs.append(float(x_value))
            ys.append(float(y_value))
    if not xs or not ys:
        return (-5.0, 5.0, -5.0, 5.0)
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    if min_x == max_x:
        min_x -= 1.0
        max_x += 1.0
    if min_y == max_y:
        min_y -= 1.0
        max_y += 1.0
    return (min_x, max_x, min_y, max_y)


def project_points(
    points: Sequence[tuple[float, float]],
    bounds: tuple[float, float, float, float],
    width: float,
    height: float,
    padding: float = 24.0,
) -> list[tuple[float, float]]:
    transform = compute_view_transform(bounds, width, height, padding=padding)
    return project_points_with_transform(points, transform)


def compute_view_transform(
    bounds: tuple[float, float, float, float],
    width: float,
    height: float,
    *,
    padding: float = 24.0,
) -> ViewTransform:
    min_x, max_x, min_y, max_y = bounds
    usable_width = max(width - 2.0 * padding, 1.0)
    usable_height = max(height - 2.0 * padding, 1.0)
    scale_x = usable_width / max(max_x - min_x, 1e-6)
    scale_y = usable_height / max(max_y - min_y, 1e-6)
    scale = min(scale_x, scale_y)
    x_offset = (width - (max_x - min_x) * scale) * 0.5
    y_offset = (height - (max_y - min_y) * scale) * 0.5
    return ViewTransform(
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
        padding=padding,
        scale=scale,
        x_offset=x_offset,
        y_offset=y_offset,
    )


def project_points_with_transform(
    points: Sequence[tuple[float, float]],
    transform: ViewTransform,
) -> list[tuple[float, float]]:
    projected: list[tuple[float, float]] = []
    for x_value, y_value in points:
        screen_x = transform.x_offset + (x_value - transform.min_x) * transform.scale
        screen_y = transform.height - (transform.y_offset + (y_value - transform.min_y) * transform.scale)
        projected.append((screen_x, screen_y))
    return projected


def screen_to_world(
    screen_x: float,
    screen_y: float,
    transform: ViewTransform,
) -> tuple[float, float]:
    world_x = (screen_x - transform.x_offset) / transform.scale + transform.min_x
    world_y = ((transform.height - screen_y) - transform.y_offset) / transform.scale + transform.min_y
    return (world_x, world_y)


def extract_occupancy_points(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    *,
    threshold: int,
    max_points: int = 4000,
) -> list[tuple[float, float]]:
    if width <= 0 or height <= 0 or resolution <= 0.0 or not data:
        return []
    total_cells = width * height
    stride = max(1, int(math.sqrt(max(total_cells / max(max_points, 1), 1.0))))
    points: list[tuple[float, float]] = []
    for row in range(0, height, stride):
        for col in range(0, width, stride):
            index = row * width + col
            if index >= len(data):
                continue
            if int(data[index]) < threshold:
                continue
            points.append(
                (
                    origin_x + (col + 0.5) * resolution,
                    origin_y + (row + 0.5) * resolution,
                )
            )
    return points


def project_series(
    values: Sequence[float],
    width: float,
    height: float,
    *,
    padding_x: float = 24.0,
    padding_y: float = 18.0,
    value_range: tuple[float, float] | None = None,
) -> list[tuple[float, float]]:
    if not values:
        return []
    if value_range is None:
        min_value = min(values)
        max_value = max(values)
    else:
        min_value, max_value = value_range
    if math.isclose(min_value, max_value):
        min_value -= 1.0
        max_value += 1.0
    usable_width = max(width - 2.0 * padding_x, 1.0)
    usable_height = max(height - 2.0 * padding_y, 1.0)
    max_index = max(len(values) - 1, 1)
    projected: list[tuple[float, float]] = []
    for index, value in enumerate(values):
        x_value = padding_x + usable_width * (index / max_index)
        y_value = height - (padding_y + usable_height * ((value - min_value) / (max_value - min_value)))
        projected.append((x_value, y_value))
    return projected
