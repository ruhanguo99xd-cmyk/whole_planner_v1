import launch
import launch_ros

import os
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# 该 launch 文件负责启动轨迹规划系统的所有节点，并根据传入的机型参数加载对应的校准参数文件。
def _launch_setup(context, *args, **kwargs):
    # 获取机型参数并验证合法性
    machine_model = LaunchConfiguration('machine_model').perform(context)
    # 目前支持的机型参数集合
    supported_models = {'prototype', 'wk35', 'wk10b'}
    # 如果传入的机型参数不在支持的集合中，则抛出异常并提示用户
    if machine_model not in supported_models:
        raise RuntimeError(
            f"Unsupported machine_model: {machine_model}. "
            f"Expected one of: {', '.join(sorted(supported_models))}"
        )

    # 参数文件路径
    calib_yaml = os.path.join(
        get_package_share_directory('launch_traplanning'),
        'config',
        'calib',
        f'{machine_model}.yaml'
    )

    # 模拟物料发送的节点
    action_node_prsdata_send  = launch_ros.actions.Node(
        package='PRSdata_send',
        executable='prsdata_server',
        name='prsdata_server',
        output='screen',
        emulate_tty=True,

        # prefix='gnome-terminal --',  #单开一个终端
    )

    # 模拟矿卡位姿发送的节点
    action_node_perceive_truck_send = launch_ros.actions.Node(
        package='PRSdata_send',
        executable='perceive_truck_server',
        name='perceive_truck_server',
        output='screen',
        emulate_tty=True,

        # prefix='gnome-terminal --',  #单开一个终端
    )
    
   # 连续动作执行节点
    action_node_plc_control  = launch_ros.actions.Node(
        package='plc_control',
        executable='plc_control_test1',
        name='plc_control',
        output='screen',
        emulate_tty=True, #允许交互操作

        # prefix='gnome-terminal --',  #单开一个终端
    )
    
    # ROS2动作客户端节点：接收挖掘动作请求并执行流程
    action_node_dig_client = launch_ros.actions.Node(
        package='PRSdata_send',
        executable='prsdata_client',
        name='dig_action_client',
        output='screen',
        emulate_tty=True,

        # prefix='gnome-terminal --',
    )

    # 挖掘规划节点：接收挖掘动作请求，调用规划系统计算轨迹并发送PLC执行
    action_node_tra_planning  = launch_ros.actions.Node(
        package='tra_planning',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
        emulate_tty=True, #允许交互操作
        parameters=[calib_yaml],   # 参数文件传入（机型几何参数在 parameters.cpp 中切换）

        # prefix='gnome-terminal --',  #单开一个终端，暂时保留交互
    )

    # 物料装载节点：接收装载动作请求，控制PLC执行装载动作
    action_node_load = launch_ros.actions.Node(
        package='load',
        executable='load',
        name='load_node',
        output='screen',
        emulate_tty=True,
        parameters=[calib_yaml],   # 参数文件传入

        # prefix='gnome-terminal --',
    )

    # 物料卸载节点：接收卸载动作请求，控制PLC执行卸载动作
    action_node_return = launch_ros.actions.Node(
        package='return',
        executable='return',
        name='return_node',
        output='screen',
        emulate_tty=True,
        parameters=[calib_yaml],   # 参数文件传入

        # prefix='gnome-terminal --',
    )

    # 合成启动动作并返回（OpaqueFunction 需要返回 action 列表）
    return [
        action_node_prsdata_send,
        action_node_perceive_truck_send,
        action_node_plc_control,
        action_node_dig_client,
        action_node_tra_planning,
        action_node_load,
        action_node_return
    ]

# 生成 launch 描述，包含一个机型参数声明和一个启动设置动作
# ros2 launch 命令可通过 machine_model:=<model> 选择标定参数文件，例如：
# ros2 launch launch_traplanning launch.py machine_model:=wk35
def generate_launch_description():
    return launch.LaunchDescription([
        # 机型参数声明（当前用于选择标定参数文件）
        DeclareLaunchArgument(
            'machine_model',
            default_value='prototype',   # 默认使用样机参数
            description='Calibration parameter set: prototype | wk35 | wk10b (trajectory_planner机型参数请在 parameters.cpp 中切换)'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
