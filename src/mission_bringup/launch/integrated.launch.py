from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def _resolve_material_target_config_yaml(root: Path, machine_model: str) -> Path:
    config_dir = root / 'config' / 'planning' / 'material_target'
    candidate = config_dir / f'{machine_model}.yaml'
    if candidate.exists():
        return candidate
    default_yaml = config_dir / 'default.yaml'
    if default_yaml.exists():
        return default_yaml
    raise RuntimeError(
        'Material target config not found. '
        f'Checked {candidate} and {default_yaml}'
    )


def _resolve_excavation_calib_yaml(root: Path, machine_model: str) -> Path:
    calib_dir = root / 'src' / 'vendor' / 'excavation_planner_core' / 'launch_traplanning' / 'config' / 'calib'
    candidate = calib_dir / f'{machine_model}.yaml'
    if candidate.exists():
        return candidate
    fallback = calib_dir / 'prototype.yaml'
    if fallback.exists():
        print(f'[integrated.launch] calibration for {machine_model} not found, fallback to {fallback.name}')
        return fallback
    raise RuntimeError(
        'Excavation calibration file not found. '
        f'Checked {candidate} and {fallback}'
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    root = _resolve_workspace_root(Path(get_package_share_directory('mission_bringup')))
    mock_sequence = root / 'config' / 'plc' / 'mock_sequence.yaml'
    machine_model = LaunchConfiguration('machine_model').perform(context)
    cedar_machine_profile = LaunchConfiguration('cedar_machine_profile').perform(context)
    cedar_plc_ip = LaunchConfiguration('cedar_plc_ip').perform(context)
    cedar_localization_mode = LaunchConfiguration('cedar_localization_mode').perform(context).strip().lower()
    cedar_odom_topic = LaunchConfiguration('cedar_odom_topic').perform(context)
    resolved_cedar_odom_topic = '/fake_odom' if cedar_localization_mode == 'fake_odom' and cedar_odom_topic == '/ekf_odom' else cedar_odom_topic
    material_boundary_extrinsic_yaml = _resolve_material_boundary_extrinsic_yaml(root, machine_model)
    material_target_config_yaml = _resolve_material_target_config_yaml(root, machine_model)
    cedar_bringup_launch = Path(get_package_share_directory('bringup')) / 'launch' / 'ekf_bringup.launch.py'
    lite_slam_config = (
        root
        / 'src'
        / 'vendor'
        / 'mobility_planner_core'
        / 'perception'
        / 'lite_slam'
        / 'config'
        / 'default.yaml'
    )
    calib_yaml = _resolve_excavation_calib_yaml(root, machine_model)

    excavation_csv_root = root / 'src' / 'vendor' / 'excavation_planner_core' / 'csv_created'
    trajectory_output_root = excavation_csv_root / 'trajectory_planner'
    load_output_root = excavation_csv_root / 'load'
    return_output_root = excavation_csv_root / 'return'
    load_output = load_output_root / 'load_fix_rotation.csv'
    return_output = return_output_root / 'return_fix_rotation.csv'

    launch_legacy_dig = _as_bool(LaunchConfiguration('launch_legacy_dig').perform(context))
    launch_legacy_dig_planner = _as_bool(LaunchConfiguration('launch_legacy_dig_planner').perform(context))
    if launch_legacy_dig and launch_legacy_dig_planner:
        raise RuntimeError('launch_legacy_dig and launch_legacy_dig_planner cannot both be true')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(cedar_bringup_launch)),
            condition=IfCondition(LaunchConfiguration('launch_cedar_walk_stack')),
            launch_arguments={
                'use_sim_time': 'false',
                'use_rviz': 'false',
                'machine_profile': cedar_machine_profile,
                'plc_ip': cedar_plc_ip,
                'localization_mode': cedar_localization_mode,
                'launch_cmd_vel_to_plc': LaunchConfiguration('cedar_launch_cmd_vel_to_plc'),
                'use_lite_slam': 'false',
                'lite_slam_config': str(lite_slam_config),
            }.items(),
        ),
        Node(
            package='plc_adapter',
            executable='plc_adapter_node',
            name='plc_adapter',
            parameters=[
                {
                    'backend': LaunchConfiguration('plc_status_backend'),
                    'command_backend': LaunchConfiguration('plc_command_backend'),
                    'mock_sequence_file': str(mock_sequence),
                    'plc_ip': cedar_plc_ip,
                    'synthetic_initial_mode': 'walk',
                    'release_swing_brake_command': 'DB1302.12040.5',
                    'engage_swing_brake_command': 'DB1302.12041.2',
                    'engage_left_walk_brake_command': 'DB1302.12041.3',
                    'engage_right_walk_brake_command': 'DB1302.12041.4',
                    'switch_dig_mode_command': 'DB1302.12041.5',
                    'switch_walk_mode_command': 'DB1302.12041.6',
                    'mode_switch_repeat_count': 2,
                }
            ],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='mobility_action_server',
            name='mobility_planner_core',
            parameters=[
                {
                    'backend': LaunchConfiguration('mobility_backend'),
                    'mock_duration_sec': 3.0,
                    'cedar_action_name': '/navigate_to_pose',
                    'cedar_goal_tolerance_m': LaunchConfiguration('cedar_goal_tolerance_m'),
                    'cedar_odom_topic': resolved_cedar_odom_topic,
                }
            ],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='machine_mode_state_bridge',
            name='machine_mode_state_bridge',
            condition=IfCondition(LaunchConfiguration('launch_machine_mode_bridge')),
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='material_target_planner',
            name='material_target_planner',
            parameters=[
                str(material_target_config_yaml),
                {
                    'enable_boundary_extractor': LaunchConfiguration('enable_boundary_extractor'),
                    'boundary_extractor_service_name': '/mobility/extract_material_boundary',
                }
            ],
            output='screen',
        ),
        Node(
            package='mobility_planner_core',
            executable='walk_scan_orchestrator',
            name='walk_scan_orchestrator',
            parameters=[
                {
                    'rotation_angle_topic': '/lite_slam/swing_angle_deg',
                    'map_topic': '/map',
                    'release_brake_service_name': '/plc_bridge/release_swing_brake',
                    'start_scan_service_name': '/plc_bridge/start_walk_scan',
                    'stop_scan_service_name': '/plc_bridge/stop_walk_scan',
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
                    'endpoint': cedar_plc_ip,
                    'db_address': 'DB1300.88',
                    'read_size_bytes': 4,
                    'value_type': 'real_deg',
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
            parameters=[{'runtime_environment': LaunchConfiguration('hmi_runtime_environment')}],
            output='screen',
        ),
        Node(
            package='planner_client',
            executable='planner_client_node',
            name='planner_client_node',
            condition=IfCondition(LaunchConfiguration('launch_cedar_walk_stack')),
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
            package='prsdata_send',
            executable='prsdata_server',
            name='prsdata_server',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            output='screen',
        ),
        Node(
            package='prsdata_send',
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
            parameters=[
                str(calib_yaml),
                {
                    'csv_output_root': str(trajectory_output_root),
                },
            ],
            output='screen',
        ),
        Node(
            package='load',
            executable='load',
            name='load_node',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            parameters=[
                str(calib_yaml),
                {
                    'csv_output_root': str(load_output_root),
                },
            ],
            output='screen',
        ),
        Node(
            package='return',
            executable='return',
            name='return_node',
            condition=IfCondition(LaunchConfiguration('launch_legacy_dig_planner')),
            parameters=[
                str(calib_yaml),
                {
                    'csv_output_root': str(return_output_root),
                },
            ],
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
        DeclareLaunchArgument('cedar_machine_profile', default_value='M001'),
        DeclareLaunchArgument('cedar_plc_ip', default_value='192.168.2.20'),
        DeclareLaunchArgument('cedar_localization_mode', default_value='ekf'),
        DeclareLaunchArgument('plc_status_backend', default_value='synthetic'),
        DeclareLaunchArgument('plc_command_backend', default_value='real'),
        DeclareLaunchArgument('mobility_backend', default_value='cedar_nav2_direct'),
        DeclareLaunchArgument('launch_cedar_walk_stack', default_value='true'),
        DeclareLaunchArgument('cedar_launch_cmd_vel_to_plc', default_value='true'),
        DeclareLaunchArgument('launch_machine_mode_bridge', default_value='true'),
        DeclareLaunchArgument('cedar_goal_tolerance_m', default_value='0.8'),
        DeclareLaunchArgument('cedar_odom_topic', default_value='/ekf_odom'),
        DeclareLaunchArgument('launch_mock_nav2', default_value='false'),
        DeclareLaunchArgument('launch_legacy_walk', default_value='false'),
        DeclareLaunchArgument('launch_legacy_dig', default_value='false'),
        DeclareLaunchArgument('launch_legacy_dig_planner', default_value='true'),
        DeclareLaunchArgument('launch_operator_hmi', default_value='false'),
        DeclareLaunchArgument('hmi_runtime_environment', default_value='field'),
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
        DeclareLaunchArgument('swing_angle_backend', default_value='plc_s7'),
        DeclareLaunchArgument('mock_swing_speed_deg_per_sec', default_value='18.0'),
        DeclareLaunchArgument('rotation_limit_deg', default_value='120.0'),
        OpaqueFunction(function=_launch_setup),
    ])
