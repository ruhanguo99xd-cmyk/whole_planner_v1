from mobility_planner_core.mock_runtime import resolve_mock_behavior


def test_default_behavior_is_success():
    behavior = resolve_mock_behavior('', 4.0)
    assert behavior.outcome == 'success'
    assert behavior.duration_sec == 4.0


def test_override_behavior_from_json():
    behavior = resolve_mock_behavior('{"simulate": "fail", "duration_sec": 1.5}', 4.0)
    assert behavior.outcome == 'fail'
    assert behavior.duration_sec == 1.5
