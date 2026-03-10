#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os
from pathlib import Path
import tempfile
import yaml


WORKSPACE_ROOT = Path('/home/ruhanguo/anew_autowalk_v3')


def _safe_load_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data if isinstance(data, dict) else {}


def _deep_update(dst: dict, src: dict) -> dict:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_update(dst[k], v)
        else:
            dst[k] = v
    return dst


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('bringup')
    ekf_config_file = os.path.join(bringup_dir, 'config', 'localization', 'ekf_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    track_width = LaunchConfiguration('track_width').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_lite_slam = LaunchConfiguration('use_lite_slam').perform(context)
    lite_slam_config = LaunchConfiguration('lite_slam_config').perform(context)
    machine_profile = LaunchConfiguration('machine_profile').perform(context)

    # -------- machine profile loading --------
    profile_path = WORKSPACE_ROOT / 'config' / 'machines' / f'{machine_profile}.yaml'
    profile = _safe_load_yaml(profile_path)

    # fallback to active profile if machine_profile file absent
    if not profile:
        active_path = WORKSPACE_ROOT / 'config' / 'machine.active.yaml'
        profile = _safe_load_yaml(active_path)

    control_cfg = profile.get('control', {}) if isinstance(profile, dict) else {}
    encoder_cfg = profile.get('encoder', {}) if isinstance(profile, dict) else {}
    rtk_cfg = profile.get('rtk', {}) if isinstance(profile, dict) else {}

    wheel_base = float(control_cfg.get('wheel_base', track_width))
    plc_ip = str(encoder_cfg.get('endpoint', '192.168.2.20'))
    rtk_host = str(rtk_cfg.get('host', '192.168.2.136'))
    rtk_port = int(rtk_cfg.get('port', 9904))
    origin_lat = float(rtk_cfg.get('origin_lat', 37.758417))
    origin_lon = float(rtk_cfg.get('origin_lon', 112.593225))
    origin_alt = float(rtk_cfg.get('origin_alt', 762.368099))

    # merge machine profile into lite_slam config (runtime generated)
    base_lite = _safe_load_yaml(Path(lite_slam_config))
    merged_lite = _deep_update(base_lite, profile if isinstance(profile, dict) else {})
    tmp_cfg = Path(tempfile.gettempdir()) / f'lite_slam_{machine_profile}.yaml'
    with open(tmp_cfg, 'w', encoding='utf-8') as f:
        yaml.safe_dump(merged_lite, f, allow_unicode=True, sort_keys=False)

    actions = []

    # 1) 通用底座
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'bringup.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'track_width': str(wheel_base),
            'map': map_file,
            'use_rviz': use_rviz
        }.items(),
    ))

    # 2) 定位链
    actions.append(Node(
        package='rtk_to_odom',
        executable='rtk_to_odom',
        name='rtk_to_odom',
        parameters=[{
            'rtk_host': rtk_host,
            'rtk_port': rtk_port,
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'origin_alt': origin_alt,
            'use_sim_time': use_sim_time,
            'publish_tf': False
        }],
        output='screen'
    ))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['6.54578', '13.8202', '0', '-1.5854', '0', '0', 'map', 'odom']
    ))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_publisher',
        arguments=['1.79319', '-0.6', '0', '0', '0', '0', 'gps_antenna_link', 'base_link']
    ))

    actions.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[('/odometry/filtered', '/ekf_odom')]
    ))

    # 3) 控制桥接（参数可由机型覆盖）
    actions.append(Node(
        package='cmd_vel_to_plc',
        executable='cmd_vel_to_plc',
        name='cmd_vel_to_plc',
        parameters=[{
            'plc_ip': plc_ip,
            'wheel_base': wheel_base,
            'forbid_in_place_spin': True,
            'min_linear_for_turn': 0.12,
            'spin_angular_threshold': 0.08,
            'min_turning_radius': 5.0,
        }],
        output='screen'
    ))

    # 4) 可选 lite_slam
    actions.append(ExecuteProcess(
        cmd=['python3', '-m', 'lite_slam.app', '--config', str(tmp_cfg), '--spin'],
        cwd='/home/ruhanguo/anew_autowalk_v3/src/perception/lite_slam',
        additional_env={
            'PYTHONPATH': '/home/ruhanguo/anew_autowalk_v3/src/perception/lite_slam/src'
        },
        condition=IfCondition(LaunchConfiguration('use_lite_slam')),
        output='screen'
    ))

    print(f"[ekf_bringup] machine_profile={machine_profile}, plc_ip={plc_ip}, wheel_base={wheel_base}")
    print(f"[ekf_bringup] rtk_host={rtk_host}:{rtk_port}, origin=({origin_lat},{origin_lon},{origin_alt})")
    print(f"[ekf_bringup] lite_slam merged config: {tmp_cfg}")

    return actions


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('track_width', default_value='2.0'))
    ld.add_action(DeclareLaunchArgument('map', default_value='/home/ruhanguo/anew_autowalk_v3/src/navigation/map_server/maps/map.yaml'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_lite_slam', default_value='false'))
    ld.add_action(DeclareLaunchArgument('lite_slam_config', default_value='/home/ruhanguo/anew_autowalk_v3/src/perception/lite_slam/config/default.yaml'))
    ld.add_action(DeclareLaunchArgument('machine_profile', default_value='M001', description='机型参数ID，如 M001~M007'))

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
