#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    goal_tolerance = LaunchConfiguration('goal_tolerance', default='0.2')
    use_planner_client = LaunchConfiguration('use_planner_client', default='false')
    navigate_action_name = LaunchConfiguration('navigate_action_name', default='navigate_to_pose')
    odom_topic = LaunchConfiguration('odom_topic', default='odom')
    
    # Get package directories
    pkg_autonomous_walk = get_package_share_directory('autonomous_walk')
    pkg_bringup = get_package_share_directory('bringup')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument('goal_tolerance', default_value=goal_tolerance, description='Goal tolerance in meters'))
    ld.add_action(DeclareLaunchArgument('use_planner_client', default_value=use_planner_client, description='Use planner client service'))
    ld.add_action(DeclareLaunchArgument('navigate_action_name', default_value=navigate_action_name, description='Navigate action name'))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic name'))
    
    # Include bringup launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_bringup, 'launch', 'bringup.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))
    
    # Autonomous walk node
    ld.add_action(Node(
        package='autonomous_walk',
        executable='autonomous_walk_node',
        name='autonomous_walk',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'goal_tolerance': goal_tolerance,
                'use_planner_client': use_planner_client,
                'navigate_action_name': navigate_action_name,
                'odom_topic': odom_topic
            }
        ]
    ))
    
    return ld
