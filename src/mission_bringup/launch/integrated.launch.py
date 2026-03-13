from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _resolve_workspace_root(package_share: Path) -> Path:
    markers = (
        Path('config/plc/mock_sequence.yaml'),
        Path('src/mission_bringup/launch/integrated.launch.py'),
    )
    for candidate in (package_share, *package_share.parents):
        if all((candidate / marker).exists() for marker in markers):
            return candidate
    raise RuntimeError(f'Unable to resolve workspace root from package share: {package_share}')


def _resolve_material_boundary_extrinsic_yaml(root: Path, machine_model: str) -> Path:
    extrinsic_dir = root / 'config' / 'perception' / 'material_boundary_extrinsic'
    candidate = extrinsic_dir / f'{machine_model}.yaml'
    if candidate.exists():
        return candidate
    default_yaml = extrinsic_dir / 'default.yaml'
    if default_yaml.exists():
        return default_yaml
    raise RuntimeError(
        'Material boundary extrinsic config not found. '
        f'Checked {candidate} and {default_yaml}'
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    root = _resolve_workspace_root(Path(get_package_share_directory('mission_bringup')))
    mock_sequence = root / 'config' / 'plc' / 'mock_sequence.yaml'
    machine_model = LaunchConfiguration('machine_model').perform(context)
    material_boundary_extrinsic_yaml = _resolve_material_boundary_extrinsic_yaml(root, machine_model)
    calib_yaml = (
        root
        / 'src'
        / 'vendor'
        / 'excavation_planner_core'
        / 'launch_traplanning'
        / 'config'
        / 'calib'
        / f'{machine_model}.yaml'
    )
    if not calib_yaml.exists():
        raise RuntimeError(f'Calibration file not found: {calib_yaml}')

    load_output = root / 'csv_created' / 'load' / 'load_fix_rotation.csv'
    return_output = root / 'csv_created' / 'return' / 'return_fix_rotation.csv'

    launch_legacy_dig = _as_bool(LaunchConfiguration('launch_legacy_dig').perform(context))
    launch_legacy_dig_planner = _as_bool(LaunchConfiguration('launch_legacy_dig_planner').perform(context))
    if launch_legacy_dig and launch_legacy_dig_planner:
        raise RuntimeError('launch_legacy_dig and launch_legacy_dig_planner cannot both be true')

    return [
        Node(
            package='plc_adapter',
            executable='plc_adapter_node',
            name='plc_adapter',
            parameters=[{'backend': 'mock', 'mock_sequence_file': str(mock_sequence)}],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='mobility_action_server',
            name='mobility_planner_core',
            parameters=[{'backend': 'legacy_autonomous_walk_service', 'mock_duration_sec': 3.0}],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='material_target_planner',
            name='material_target_planner',
            parameters=[
                {
                    'enable_boundary_extractor': LaunchConfiguration('enable_boundary_extractor'),
                    'boundary_extractor_service_name': '/mobility/extract_material_boundary',
                }
            ],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='material_boundary_extractor',
            name='material_boundary_extractor',
            condition=IfCondition(LaunchConfiguration('launch_material_boundary_extractor')),
            parameters=[
                str(material_boundary_extrinsic_yaml),
                {
                    'point_cloud_topic': LaunchConfiguration('material_point_cloud_topic'),
                    'service_name': '/mobility/extract_material_boundary',
                    'boundary_topic': '/mobility/material_boundary',
                    'debug_topic': '/mobility/material_boundary_debug',
                    'use_static_extrinsic': LaunchConfiguration('use_boundary_static_extrinsic'),
                    'sensor_frame_id': LaunchConfiguration('boundary_sensor_frame_id'),
                    'extrinsic_translation_x_m': LaunchConfiguration('boundary_extrinsic_translation_x_m'),
                    'extrinsic_translation_y_m': LaunchConfiguration('boundary_extrinsic_translation_y_m'),
                    'extrinsic_translation_z_m': LaunchConfiguration('boundary_extrinsic_translation_z_m'),
                    'extrinsic_roll_deg': LaunchConfiguration('boundary_extrinsic_roll_deg'),
                    'extrinsic_pitch_deg': LaunchConfiguration('boundary_extrinsic_pitch_deg'),
                    'extrinsic_yaw_deg': LaunchConfiguration('boundary_extrinsic_yaw_deg'),
                }
            ],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='lite_slam_swing_angle_bridge',
            name='lite_slam_swing_angle_bridge',
            condition=IfCondition(LaunchConfiguration('launch_swing_angle_bridge')),
            parameters=[
                {
                    'backend': LaunchConfiguration('swing_angle_backend'),
                    'mock_rotation_speed_deg_per_sec': LaunchConfiguration('mock_swing_speed_deg_per_sec'),
                }
            ],
            output='screen',
        ),
        Node(
            package='excavation_planner_core',
            executable='excavation_action_server',
            name='excavation_planner_core',
            parameters=[{'backend': 'legacy_dig_command', 'mock_duration_sec': 4.0}],
            output='screen',
        ),
        Node(
            package='mission_dispatcher',
            executable='mission_dispatcher_node',
            name='mission_dispatcher',
            parameters=[{'auto_start': True, 'walk_timeout_sec': 15.0, 'dig_timeout_sec': 90.0}],
            output='screen',
        ),
        Node(
            package='mission_operator_hmi',
            executable='integrated_operator_hmi',
            name='integrated_operator_hmi',
            condition=IfCondition(LaunchConfiguration('launch_operator_hmi')),
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='mock_nav2_server',
            name='mock_nav2_server',
            condition=IfCondition(LaunchConfiguration('launch_mock_nav2')),
            output='screen',
        ),
        Node(
            package='autonomous_walk',
            executable='autonomous_walk_node',
            name='autonomous_walk',
            condition=IfCondition(LaunchConfiguration('launch_legacy_walk')),
            output='screen',
        ),
        Node(
            package='excavation_planner_core',
            executable='legacy_perception_notifier',
            name='legacy_perception_notifier',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig')),
            output='screen',
        ),
        Node(
            package='plc_control',
            executable='plc_control_test1',
            name='plc_control',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig')),
            output='screen',
        ),
        Node(
            package='PRSdata_send',
            executable='prsdata_server',
            name='prsdata_server',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            output='screen',
        ),
        Node(
            package='PRSdata_send',
            executable='perceive_truck_server',
            name='perceive_truck_server',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            output='screen',
        ),
        Node(
            package='tra_planning',
            executable='trajectory_planner',
            name='trajectory_planner',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            parameters=[str(calib_yaml)],
            output='screen',
        ),
        Node(
            package='load',
            executable='load',
            name='load_node',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            parameters=[str(calib_yaml)],
            output='screen',
        ),
        Node(
            package='return',
            executable='return',
            name='return_node',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            parameters=[str(calib_yaml)],
            output='screen',
        ),
        Node(
            package='excavation_planner_core',
            executable='legacy_dig_planner_orchestrator',
            name='legacy_dig_planner_orchestrator',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            parameters=[
                {
                    'load_output_file': str(load_output),
                    'return_output_file': str(return_output),
                    'perception_timeout_sec': 20.0,
                    'planner_timeout_sec': 90.0,
                    'load_return_timeout_sec': 90.0,
                    'max_consecutive_no_result': 3,
                    'rotation_limit_deg': LaunchConfiguration('rotation_limit_deg'),
                    'rotation_angle_topic': '/lite_slam/swing_angle_deg',
                    'cycle_pause_sec': 0.5,
                }
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('machine_model', default_value='prototype'),
        DeclareLaunchArgument('launch_mock_nav2', default_value='true'),
        DeclareLaunchArgument('launch_legacy_walk', default_value='true'),
        DeclareLaunchArgument('launch_legacy_dig', default_value='false'),
        DeclareLaunchArgument('launch_legacy_dig_planner', default_value='true'),
        DeclareLaunchArgument('launch_operator_hmi', default_value='false'),
        DeclareLaunchArgument('launch_material_boundary_extractor', default_value='true'),
        DeclareLaunchArgument('enable_boundary_extractor', default_value='true'),
        DeclareLaunchArgument('material_point_cloud_topic', default_value='/material/point_cloud'),
        DeclareLaunchArgument('use_boundary_static_extrinsic', default_value='false'),
        DeclareLaunchArgument('boundary_sensor_frame_id', default_value=''),
        DeclareLaunchArgument('boundary_extrinsic_translation_x_m', default_value='0.0'),
        DeclareLaunchArgument('boundary_extrinsic_translation_y_m', default_value='0.0'),
        DeclareLaunchArgument('boundary_extrinsic_translation_z_m', default_value='0.0'),
        DeclareLaunchArgument('boundary_extrinsic_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('boundary_extrinsic_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('boundary_extrinsic_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('launch_swing_angle_bridge', default_value='true'),
        DeclareLaunchArgument('swing_angle_backend', default_value='mock'),
        DeclareLaunchArgument('mock_swing_speed_deg_per_sec', default_value='18.0'),
        DeclareLaunchArgument('rotation_limit_deg', default_value='120.0'),
        OpaqueFunction(function=_launch_setup),
    ])
