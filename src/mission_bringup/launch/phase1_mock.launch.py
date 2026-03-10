from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    root = Path(get_package_share_directory('mission_bringup')).parents[3]
    mock_sequence = root / 'config' / 'plc' / 'mock_sequence.yaml'
    return LaunchDescription([
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
            parameters=[{'backend': 'mock', 'mock_duration_sec': 3.0}],
            output='screen',
        ),
        Node(
            package='excavation_planner_core',
            executable='excavation_action_server',
            name='excavation_planner_core',
            parameters=[{'backend': 'mock', 'mock_duration_sec': 4.0}],
            output='screen',
        ),
        Node(
            package='mission_dispatcher',
            executable='mission_dispatcher_node',
            name='mission_dispatcher',
            parameters=[{'auto_start': True, 'walk_timeout_sec': 10.0, 'dig_timeout_sec': 12.0}],
            output='screen',
        ),
    ])
