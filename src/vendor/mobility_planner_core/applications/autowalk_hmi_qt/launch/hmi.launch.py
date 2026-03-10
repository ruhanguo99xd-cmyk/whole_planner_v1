#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    track_width = LaunchConfiguration('track_width')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time and prefer /fake_odom for pose source',
        ),
        DeclareLaunchArgument(
            'track_width',
            default_value='1.925',
            description='Track width (m) for left/right track speed computation',
        ),
        Node(
            package='autowalk_hmi_qt',
            executable='autowalk_hmi_qt',
            parameters=[{
                'use_sim_time': use_sim_time,
                'track_width': track_width,
            }],
            output='screen'
        )
    ])
