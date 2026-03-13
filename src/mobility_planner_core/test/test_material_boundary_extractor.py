import json

from geometry_msgs.msg import PoseStamped

import pytest

from mobility_planner_core.material_boundary_extractor import (
    StaticExtrinsicConfig,
    extract_boundary_from_xyz_points,
    transform_xyz_points_to_target_frame,
)
from mobility_planner_core.material_target_planner import should_use_pointcloud_extractor


def _pose(x: float, y: float, frame_id: str = 'map') -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def test_extract_boundary_from_xyz_points_filters_z_and_generates_outline() -> None:
    current = _pose(0.0, -6.0)
    material = _pose(0.0, 0.0)
    xyz_points = (
        (-2.0, 0.0, 0.2),
        (-1.4, -1.4, 0.2),
        (0.0, -2.0, 0.2),
        (1.4, -1.4, 0.2),
        (2.0, 0.0, 0.2),
        (1.4, 1.4, 0.2),
        (0.0, 2.0, 0.2),
        (-1.4, 1.4, 0.2),
        (0.0, 0.0, 3.5),
    )

    outline, debug = extract_boundary_from_xyz_points(
        xyz_points=xyz_points,
        cloud_frame_id='map',
        current_pose=current,
        material_reference_pose=material,
        profile={},
        constraints={
            'boundary_input': {
                'z_min_m': -0.5,
                'z_max_m': 1.0,
                'angular_bins': 16,
                'min_boundary_radius_m': 0.5,
            }
        },
    )

    assert len(outline) >= 8
    assert debug['z_filtered_count'] == 8
    assert debug['status'] == 'ok'


def test_extract_boundary_from_xyz_points_respects_roi() -> None:
    current = _pose(0.0, -6.0)
    material = _pose(0.0, 0.0)
    xyz_points = (
        (-1.0, -1.0, 0.0),
        (1.0, -1.0, 0.0),
        (1.0, 1.0, 0.0),
        (-1.0, 1.0, 0.0),
        (8.0, 8.0, 0.0),
    )

    outline, debug = extract_boundary_from_xyz_points(
        xyz_points=xyz_points,
        cloud_frame_id='map',
        current_pose=current,
        material_reference_pose=material,
        profile={},
        constraints={
            'boundary_input': {
                'roi_center': 'material_reference_pose',
                'roi_radius_m': 3.0,
                'angular_bins': 12,
            }
        },
    )

    assert len(outline) == 4
    assert debug['scatter_outline']['roi_filtered_count'] == 4


def test_transform_xyz_points_to_target_frame_identity_when_frames_match() -> None:
    transformed, debug = transform_xyz_points_to_target_frame(
        xyz_points=((1.0, 2.0, 3.0),),
        cloud_frame_id='map',
        target_frame_id='map',
        static_extrinsic=StaticExtrinsicConfig(),
    )

    assert transformed == ((1.0, 2.0, 3.0),)
    assert debug['mode'] == 'identity_same_frame'


def test_transform_xyz_points_to_target_frame_uses_static_extrinsic() -> None:
    transformed, debug = transform_xyz_points_to_target_frame(
        xyz_points=((1.0, 0.0, 0.0),),
        cloud_frame_id='lidar_frame',
        target_frame_id='map',
        static_extrinsic=StaticExtrinsicConfig(
            enabled=True,
            sensor_frame_id='lidar_frame',
            translation_x_m=10.0,
            translation_y_m=1.0,
            yaw_deg=90.0,
        ),
    )

    assert transformed[0][0] == pytest.approx(10.0, abs=1e-6)
    assert transformed[0][1] == pytest.approx(2.0, abs=1e-6)
    assert debug['mode'] == 'static_extrinsic'


def test_transform_xyz_points_to_target_frame_fails_without_extrinsic() -> None:
    with pytest.raises(ValueError, match='frame mismatch without tf or static extrinsic'):
        transform_xyz_points_to_target_frame(
            xyz_points=((1.0, 0.0, 0.0),),
            cloud_frame_id='lidar_frame',
            target_frame_id='map',
            static_extrinsic=StaticExtrinsicConfig(),
        )


def test_extract_boundary_from_xyz_points_fails_on_frame_mismatch_without_extrinsic() -> None:
    current = _pose(0.0, -6.0)
    material = _pose(0.0, 0.0)

    with pytest.raises(ValueError, match='frame mismatch without tf or static extrinsic'):
        extract_boundary_from_xyz_points(
            xyz_points=((-2.0, 0.0, 0.2), (2.0, 0.0, 0.2), (0.0, 2.0, 0.2)),
            cloud_frame_id='lidar_frame',
            current_pose=current,
            material_reference_pose=material,
            profile={},
            constraints={},
        )


def test_should_use_pointcloud_extractor_when_requested() -> None:
    assert should_use_pointcloud_extractor(
        material_outline=[],
        material_profile_json='{}',
        planner_constraints_json=json.dumps({'boundary_input': {'source': 'pointcloud'}}),
    )

    assert should_use_pointcloud_extractor(
        material_outline=[],
        material_profile_json=json.dumps({'boundary_input': {'use_pointcloud_extractor': True}}),
        planner_constraints_json='{}',
    )

    assert not should_use_pointcloud_extractor(
        material_outline=[],
        material_profile_json='{}',
        planner_constraints_json='{}',
    )
