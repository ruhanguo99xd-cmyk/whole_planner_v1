from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    default_map = os.path.join(
        get_package_share_directory('map_server'),
        'maps',
        'map.yaml'
    )

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Path to map yaml file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}]
        )
    ])
