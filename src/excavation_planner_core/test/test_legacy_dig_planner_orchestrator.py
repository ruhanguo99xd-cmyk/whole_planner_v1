from pathlib import Path

from excavation_planner_core.legacy_dig_planner_orchestrator import LegacyDigPlannerOrchestrator


def test_path_updated_after_detects_missing_file(tmp_path: Path) -> None:
    missing = tmp_path / 'missing.csv'
    assert not LegacyDigPlannerOrchestrator._path_updated_after(missing, 1)


def test_path_updated_after_uses_mtime_ns(tmp_path: Path) -> None:
    target = tmp_path / 'planner.csv'
    target.write_text('first\n', encoding='utf-8')
    first_mtime = target.stat().st_mtime_ns
    assert LegacyDigPlannerOrchestrator._path_updated_after(target, first_mtime)
    assert not LegacyDigPlannerOrchestrator._path_updated_after(target, first_mtime + 10**9)
