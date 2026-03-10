import json
from dataclasses import dataclass


@dataclass
class MockBehavior:
    outcome: str = 'success'
    duration_sec: float = 5.0
    material_volume: float = 0.0


def resolve_mock_behavior(raw: str, default_duration_sec: float) -> MockBehavior:
    if not raw:
        return MockBehavior(duration_sec=default_duration_sec)
    payload = json.loads(raw)
    return MockBehavior(
        outcome=str(payload.get('simulate', 'success')),
        duration_sec=float(payload.get('duration_sec', default_duration_sec)),
        material_volume=float(payload.get('material_volume', 0.0)),
    )
