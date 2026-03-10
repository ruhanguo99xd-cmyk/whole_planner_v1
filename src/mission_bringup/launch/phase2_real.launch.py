from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    root = Path(get_package_share_directory('mission_bringup')).parents[3]
    mock_sequence = root / 'config' / 'plc' / 'mock_sequence.yaml'
    machine_model = LaunchConfiguration('machine_model').perform(context)
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
        OpaqueFunction(function=_launch_setup),
    ])
