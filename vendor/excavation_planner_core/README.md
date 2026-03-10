## 项目简介

本项目包含了挖掘作业规划与 PLC 联动相关的多个节点。时序图描述了感知、规划、装载/卸载及 PLC 控制的主要交互流程。

## 快速开始

```bash
cd ros2_ws

# 构建
colcon build --symlink-install

# 使能环境
source install/setup.bash

# 启动整套节点（默认使用 prototype 标定参数）
ros2 launch launch_traplanning launch.py

# 启动界面
ros2 run app app
```

## 机型参数与标定参数的切换方式（当前方案）

当前项目采用“两层配置”：

- 机型几何/约束参数：`tra_planning` 内 C++ 分文件切换（不走 ROS 参数）
- 标定参数（推压/提升零位、减速比等）：`launch_traplanning/config/calib/*.yaml`，通过 `launch.py` 的 `machine_model` 切换

### 1) 切换 `trajectory_planner` 的机型几何参数（C++）

编辑：

- `src/tra_planning/src/parameters.cpp`

修改 `#include` 行（当前只有这一处切换点）：

```cpp
#include "machine_profiles/prototype.inc"
// 可改为：
// #include "machine_profiles/wk35.inc"
// #include "machine_profiles/wk10b.inc"
```

机型参数文件位置：

- `src/tra_planning/src/machine_profiles/prototype.inc`
- `src/tra_planning/src/machine_profiles/wk35.inc`
- `src/tra_planning/src/machine_profiles/wk10b.inc`（已创建，占位参数，需后续补全）

修改后需要重新编译 `tra_planning`。

### 2) 切换标定参数（Launch 参数）

`launch.py` 中的 `machine_model` 只控制标定参数文件：

- `src/launch_traplanning/config/calib/prototype.yaml`
- `src/launch_traplanning/config/calib/wk35.yaml`
- `src/launch_traplanning/config/calib/wk10b.yaml`

示例：

```bash
# 使用 wk35 标定参数
ros2 launch launch_traplanning launch.py machine_model:=wk35

# 使用 wk10b 标定参数
ros2 launch launch_traplanning launch.py machine_model:=wk10b
```

说明：

- 如果不传 `machine_model`，默认使用 `prototype`
- `machine_model` 不会自动切换 `trajectory_planner` 的机型几何参数（该部分由 `parameters.cpp` 控制）

## 包与节点

- `PRSdata_send`：`prsdata_server`、`perceive_truck_server`
- `plc_control`：`plc_control_test1`（启动文件内使用）
- `tra_planning`：`trajectory_planner`
- `load`：`load`
- `return`：`return`
- `launch_traplanning`：统一启动与配置入口

## 包功能概述

- `tra_planning`：接收感知结果并生成挖掘轨迹 CSV，发布感知完成与轨迹长度等信息。
- `plc_control`：与 PLC 通讯，触发回转、挖掘、卸载、复位等动作流程。
- `PRSdata_send`：（模拟）对外提供点云处理与矿卡感知服务，作为规划与装载/卸载的数据源。
- `load` / `return`：根据矿卡位姿生成装载/复位阶段的 CSV 轨迹，并与 PLC 协同执行。
- `launch_traplanning`：集中启动上述节点并下发统一标定参数。

## 配置

`launch_traplanning` 会为 `tra_planning`、`load`、`return` 传入统一的标定参数文件（按 `machine_model` 选择）：

- `src/launch_traplanning/config/calib/prototype.yaml`
- `src/launch_traplanning/config/calib/wk35.yaml`
- `src/launch_traplanning/config/calib/wk10b.yaml`

## 关键话题/服务（来自时序图）

- `/process_pointcloud`（PRS 点云处理服务）
- `/perceive_truck`（矿卡位姿/物料感知服务）
- `digging/perception_start`、`digging/perception_finish`
- `digging/excavation_start`、`digging/excavation_end_length`

## 时序图

``` mermaid
  sequenceDiagram
      participant PLC
      participant PLCCTRL as plc_control_test2
      participant TP as tra_planning/trajectory_planner
      participant PRS as PRSdata_send/prsdata_server
      participant LOAD as load/load_node
      participant RETURN as return/return_node
      participant TRUCK as PRSdata_send/perceive_truck_server

      Note over PLCCTRL: 预动作阶段
      PLCCTRL->>PLC: 回转90°(rotate_planning_function)
      PLC-->>PLCCTRL: 完成位=1

      PLCCTRL->>TP: 发布 digging/perception_start
      TP->>PRS: /process_pointcloud (Wuliaoprocess.srv)
      PRS-->>TP: PRS数据响应

      alt 规划成功
          TP-->>PLCCTRL: 发布 digging/perception_finish
          Note over TP: perception_finish 在规划计算前发布
          TP-->>TP: initialize_speed_data() 生成挖掘轨迹 CSV
          TP-->>LOAD: 发布 digging/excavation_end_length(缓存)
          TP-->>RETURN: 发布 digging/excavation_end_length(缓存)
      else 规划失败
          TP-->>TP: handle_plan_failure() 回退起止点
          TP-->>PLCCTRL: 仍发布 digging/perception_finish
          TP-->>LOAD: 发布 digging/excavation_end_length(回退/历史,缓存)
          TP-->>RETURN: 发布 digging/excavation_end_length(回退/历史,缓存)
      end

      PLCCTRL->>PLC: 回转0°(rotate_planning_function)
      PLC-->>PLCCTRL: 完成位=1
      PLCCTRL->>PLC: 复位规划(reset_planning_function)
      PLC-->>PLCCTRL: 完成位=1

      loop 每一轮作业
          Note over PLCCTRL: 挖掘动作阶段
          PLCCTRL->>LOAD: 发布 digging/excavation_start
          PLCCTRL->>RETURN: 发布 digging/excavation_start
          PLCCTRL->>PLC: 写入挖掘轨迹(trajectory_planner CSV)并触发

          Note over LOAD,RETURN: 只有收到 excavation_start 才请求矿卡位姿
          LOAD->>TRUCK: /perceive_truck
          TRUCK-->>LOAD: 矿卡位姿/物料信息
          RETURN->>TRUCK: /perceive_truck
          TRUCK-->>RETURN: 矿卡位姿/物料信息

          Note over LOAD,RETURN: CSV 生成是异步的，plc_control 不等待
          LOAD-->>PLCCTRL: load CSV可读
          RETURN-->>PLCCTRL: return CSV可读

          PLC-->>PLCCTRL: 完成位=1
          Note over PLCCTRL: 完成位=1 并不保证 CSV 已生成

          Note over PLCCTRL: 卸载动作(代码未等待 CSV 就绪)
          PLCCTRL->>PLC: 写入卸载轨迹(load CSV)并触发
          PLC-->>PLCCTRL: 完成位=1

          Note over PLCCTRL: 开斗动作
          PLCCTRL->>PLC: 开斗控制位=1
          PLCCTRL->>PLC: 开斗控制位=0

          Note over PLCCTRL: 感知与规划(为下一轮缓存 excavation_end_length)
          PLCCTRL->>TP: 发布 digging/perception_start
          TP->>PRS: /process_pointcloud
          PRS-->>TP: PRS数据响应

          alt 规划成功
              TP-->>PLCCTRL: 发布 digging/perception_finish
              Note over TP: perception_finish 仍在规划计算前发布
              TP-->>TP: initialize_speed_data() 生成挖掘轨迹 CSV
              TP-->>LOAD: 发布 digging/excavation_end_length(缓存)
              TP-->>RETURN: 发布 digging/excavation_end_length(缓存)
          else 规划失败
              TP-->>TP: handle_plan_failure() 回退起止点
              TP-->>PLCCTRL: 仍发布 digging/perception_finish
              TP-->>LOAD: 发布 digging/excavation_end_length(回退/历史,缓存)
              TP-->>RETURN: 发布 digging/excavation_end_length(回退/历史,缓存)
          end

          Note over LOAD,RETURN: excavation_end_length 可能是缓存/历史

          Note over PLCCTRL: 卸载复位
          PLCCTRL->>PLC: 写入复位轨迹(return CSV)并触发
          PLC-->>PLCCTRL: 完成位=1

          Note over PLCCTRL: 感知等待有超时(10s)
      end

```
