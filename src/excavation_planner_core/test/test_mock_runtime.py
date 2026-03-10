from excavation_planner_core.mock_runtime import resolve_mock_behavior


def test_default_behavior_is_success():
    behavior = resolve_mock_behavior('', 5.0)
    assert behavior.outcome == 'success'
    assert behavior.duration_sec == 5.0


def test_override_behavior_from_json():
    behavior = resolve_mock_behavior('{"simulate": "timeout", "duration_sec": 2.0, "material_volume": 3.5}', 5.0)
    assert behavior.outcome == 'timeout'
    assert behavior.duration_sec == 2.0
    assert behavior.material_volume == 3.5
