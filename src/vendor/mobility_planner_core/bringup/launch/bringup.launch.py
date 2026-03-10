#!/usr/bin/env python3
# 文件名: bringup.launch.py
# 这是你的“通用底座”，负责 Nav2、URDF、记录器

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition # <--- 新增：用于控制 Rviz 是否启动

def generate_launch_description():
    # 1. 获取路径
    bringup_dir = get_package_share_directory('bringup')
    # 默认地图路径
    default_map = os.path.join(get_package_share_directory('map_server'), 'maps', 'map.yaml') # 假设路径
    
    # 2. 接收外部参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    track_width = LaunchConfiguration('track_width')
    map_yaml = LaunchConfiguration('map') # <--- 接收地图路径
    use_rviz = LaunchConfiguration('use_rviz') # <--- 接收是否启动 Rviz

    params_file = os.path.join(bringup_dir, 'config', 'nav2', 'nav2_params.yaml')
    rviz_config_path = os.path.join(bringup_dir, 'config', 'rviz2', 'autowalk_nav.rviz')
    urdf_file = os.path.join(get_package_share_directory('description'), 'urdf', 'dual_track.urdf.xacro')
    bt_xml = os.path.join(bringup_dir, 'config', 'nav2', 'bt', 'walking.xml')

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    ld = LaunchDescription()
    
    # --- 声明参数 (让父 Launch 文件可以修改这些值) ---
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false')) # 真机默认为 false
    ld.add_action(DeclareLaunchArgument('track_width', default_value='2.0'))
    ld.add_action(DeclareLaunchArgument('map', default_value=default_map)) # 暴露地图参数
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true')) # 默认启动 Rviz

    # --- 节点定义 ---
    
    # 1. Robot State Publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        output='screen'
    ))

    # 2. Rviz2 (带条件判断)
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz), # 只有 use_rviz=true 时才启动
        output='screen'
    ))

    # 3. 记录器节点 (保持不变)
    ld.add_action(Node(
        package='path_to_csv',
        executable='path_to_csv_node',
        parameters=[{'csv_dir': '/home/ruhanguo/anew_autowalk_v3/src/recorders/path_to_csv/csv'}],
        output='screen'
    ))
    ld.add_action(Node(
        package='cmd_vel_recorder',
        executable='cmd_vel_recorder',
        parameters=[{'csv_dir': '/home/ruhanguo/anew_autowalk_v3/src/recorders/cmd_vel_recorder/csv', 'track_width': track_width}],
        output='screen'
    ))

    # 4. Nav2 核心节点
    # 注意：map_server 需要显式接收 yaml_filename
    managed_nodes = ['map_server', 'planner_server', 'controller_server', 'bt_navigator', 'behavior_server']
    
    nav2_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            # 关键修改：在这里强制覆盖 yaml_filename
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}], 
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    # Keep multiple key names for Nav2 version compatibility.
                    'bt_xml_filename': bt_xml,
                    'default_bt_xml_filename': bt_xml,
                    'default_nav_to_pose_bt_xml': bt_xml,
                    'default_nav_through_poses_bt_xml': bt_xml,
                }
            ],
            output='screen'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        # ... (其他 Nav2 节点保持不变，省略以节省空间) ...
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': managed_nodes}],
            output='screen'
        )
    ]

    for node in nav2_nodes:
        ld.add_action(node)

    return ld
