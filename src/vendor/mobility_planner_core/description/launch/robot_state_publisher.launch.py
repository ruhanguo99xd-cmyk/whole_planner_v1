#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(
        get_package_share_directory('description'),
        'urdf', 'dual_track.urdf.xacro'))
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument('urdf_file', default_value=urdf_file, description='URDF file path'))
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', urdf_file])}
        ]
    )
    
    ld.add_action(robot_state_publisher_node)
    
    return ld
