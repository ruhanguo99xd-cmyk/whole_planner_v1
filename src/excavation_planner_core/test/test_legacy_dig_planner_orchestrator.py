from pathlib import Path

from excavation_planner_core.legacy_dig_planner_orchestrator import (
    LegacyDigPlannerOrchestrator,
    normalize_relative_rotation_deg,
    should_finish_due_to_no_result,
    should_finish_due_to_rotation,
)


def test_path_updated_after_detects_missing_file(tmp_path: Path) -> None:
    missing = tmp_path / 'missing.csv'
    assert not LegacyDigPlannerOrchestrator._path_updated_after(missing, 1)


def test_path_updated_after_uses_mtime_ns(tmp_path: Path) -> None:
    target = tmp_path / 'planner.csv'
    target.write_text('first\n', encoding='utf-8')
    first_mtime = target.stat().st_mtime_ns
    assert LegacyDigPlannerOrchestrator._path_updated_after(target, first_mtime)
    assert not LegacyDigPlannerOrchestrator._path_updated_after(target, first_mtime + 10**9)


def test_normalize_relative_rotation_deg_handles_wraparound() -> None:
    assert normalize_relative_rotation_deg(5.0, 355.0) == 10.0
    assert normalize_relative_rotation_deg(355.0, 5.0) == -10.0


def test_should_finish_due_to_rotation_uses_absolute_relative_angle() -> None:
    assert should_finish_due_to_rotation(current_deg=130.0, baseline_deg=0.0, limit_deg=120.0)
    assert should_finish_due_to_rotation(current_deg=230.0, baseline_deg=350.0, limit_deg=120.0)
    assert not should_finish_due_to_rotation(current_deg=80.0, baseline_deg=0.0, limit_deg=120.0)


def test_should_finish_due_to_no_result_respects_limit() -> None:
    assert not should_finish_due_to_no_result(2, 3)
    assert should_finish_due_to_no_result(3, 3)
