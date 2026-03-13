from pathlib import Path

import pytest

from mission_operator_hmi.helpers import (
    compute_view_transform,
    compute_bounds,
    downsample_points,
    extract_occupancy_points,
    load_numeric_series_csv,
    load_xyz_csv,
    normalize_json_text,
    parse_outline_text,
    project_points,
    project_points_with_transform,
    project_series,
    screen_to_world,
)


def test_parse_outline_text_accepts_simple_pairs() -> None:
    assert parse_outline_text('1.0,2.0\n3.5, 4.5') == [(1.0, 2.0), (3.5, 4.5)]


def test_normalize_json_text_returns_compact_json() -> None:
    assert normalize_json_text('{"a": 1, "b": [2, 3]}') == '{"a":1,"b":[2,3]}'


def test_load_numeric_series_csv_reads_first_line_series(tmp_path: Path) -> None:
    path = tmp_path / 'series.csv'
    path.write_text('1.0, 2.5, 3.0\n', encoding='utf-8')
    assert load_numeric_series_csv(path) == [1.0, 2.5, 3.0]


def test_load_xyz_csv_reads_three_column_rows(tmp_path: Path) -> None:
    path = tmp_path / 'traj.csv'
    path.write_text('1,2,3\n4,5,6\n', encoding='utf-8')
    assert load_xyz_csv(path) == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]


def test_downsample_points_keeps_last_point() -> None:
    source = [(float(index), 0.0) for index in range(1000)]
    sampled = downsample_points(source, max_points=50)
    assert sampled[-1] == source[-1]
    assert len(sampled) <= 51


def test_project_points_returns_canvas_coordinates() -> None:
    projected = project_points([(0.0, 0.0), (10.0, 10.0)], compute_bounds([[(0.0, 0.0), (10.0, 10.0)]]), 200.0, 100.0)
    assert len(projected) == 2
    for x_value, y_value in projected:
        assert 0.0 <= x_value <= 200.0
        assert 0.0 <= y_value <= 100.0


def test_screen_to_world_inverts_projection() -> None:
    bounds = compute_bounds([[(0.0, 0.0), (10.0, 10.0)]])
    transform = compute_view_transform(bounds, 200.0, 100.0)
    projected = project_points_with_transform([(2.5, 7.5)], transform)
    restored = screen_to_world(projected[0][0], projected[0][1], transform)
    assert restored[0] == pytest.approx(2.5, rel=1e-6, abs=1e-6)
    assert restored[1] == pytest.approx(7.5, rel=1e-6, abs=1e-6)


def test_extract_occupancy_points_returns_cell_centers() -> None:
    points = extract_occupancy_points(
        [0, 100, 0, 80],
        2,
        2,
        0.5,
        1.0,
        -1.0,
        threshold=50,
        max_points=10,
    )
    assert points == [(1.75, -0.75), (1.75, -0.25)]


def test_project_series_returns_monotonic_x() -> None:
    projected = project_series([0.0, 1.0, 0.5], 300.0, 120.0)
    assert len(projected) == 3
    assert projected[0][0] < projected[1][0] < projected[2][0]
