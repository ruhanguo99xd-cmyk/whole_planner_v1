# integrated_mission_planner

工程目录名：`whole_planner_v1`

这是一个把行走规划、挖掘规划、上层大调度、PLC 状态接入和统一上位机整合到一起的 ROS 2 工程。

## 仓库目标

- 融合 `anew_autowalk_v3` 行走规划能力
- 融合 `tra_planning0226` 挖掘规划能力
- 建立 walk / dig 两条 action 调度链
- 建立上层任务状态机和 PLC 驱动条件
- 提供统一上位机界面
- 支持停靠点计算与点云边界提取

## 当前能力

- 大规划状态机：`IDLE / WALK_PREP / WALKING / DIG_PREP / DIGGING / TRANSITION / FAULT`
- walk 完成条件：到达目标后做距离复核
- dig 完成条件：
  - 连续 3 次无有效规划结果
  - 或回转相对角达到阈值
- dig cancel 已下沉到：
  - `legacy_dig_planner_orchestrator`
  - `trajectory_planner`
  - `load`
  - `return`
- 停靠点规划链：
  - `boundary_fit`
  - `work_band`
  - `candidate_evaluation`
- 点云边界提取链：
  - `PointCloud2 -> material_boundary_extractor -> material_target_planner`
- 统一上位机：
  - `大规划 / 行走规划 / 挖掘规划` 三页签
  - 行走页支持地图点选目标
  - 挖掘页支持实时轨迹和优化中间态可视化

## 系统架构图

```mermaid
flowchart TB
    EXT[外部/HMI/调度系统] --> SUBMIT[/mission_dispatcher/submit_mission]
    EXT --> START[/mission_dispatcher/start]
    EXT --> STOP[/mission_dispatcher/stop]
    EXT --> RECOVER[/mission_dispatcher/recover]

    PLC[plc_adapter\n/plc/status] --> DISP[mission_dispatcher\n上层任务调度]

    SUBMIT --> DISP
    START --> DISP
    STOP --> DISP
    RECOVER --> DISP

    DISP -->|WalkMission START/STOP/CANCEL| WACT[mobility_action_server]
    DISP -->|DigMission START/STOP/CANCEL| DACT[excavation_action_server]

    DISP -->|use_material_target=true| MTP[/mobility/compute_material_target]
    MTP --> MTP_NODE[material_target_planner]

    MTP_NODE -->|需要点云边界时| EXTSVC[/mobility/extract_material_boundary]
    EXTSVC --> EXT_NODE[material_boundary_extractor]
    PC[PointCloud2] --> EXT_NODE

    WACT -->|service| AW[autonomous_walk]
    AW --> NAV[mock_nav2 / nav2]
    NAV --> AW
    AW -->|legacy status| WACT
    WACT -->|/mobility/status| DISP

    DACT -->|/dig/start /dig/stop| DORCH[legacy_dig_planner_orchestrator]
    DORCH --> TRAJ[trajectory_planner]
    DORCH --> LOAD[load]
    DORCH --> RET[return]
    PRS[prsdata_server] --> TRAJ
    TRUCK[perceive_truck_server] --> TRAJ
    ANGLE[lite_slam_swing_angle_bridge\n/lite_slam/swing_angle_deg] --> DORCH
    DORCH -->|/excavation/status| DACT
    DACT -->|/excavation/status| DISP
```

## 启动关系图

```mermaid
flowchart TB
    LAUNCH[phase2_real.launch.py] --> PLC[plc_adapter]
    LAUNCH --> DISP[mission_dispatcher]
    LAUNCH --> WACT[mobility_action_server]
    LAUNCH --> MTP[material_target_planner]
    LAUNCH --> MBE[material_boundary_extractor]
    LAUNCH --> SWING[lite_slam_swing_angle_bridge]
    LAUNCH --> DACT[excavation_action_server]
    LAUNCH --> NAV[mock_nav2_server]
    LAUNCH --> AW[autonomous_walk]

    LAUNCH --> PRS[prsdata_server]
    LAUNCH --> TRUCK[perceive_truck_server]
    LAUNCH --> TRAJ[trajectory_planner]
    LAUNCH --> LOAD[load]
    LAUNCH --> RET[return]
    LAUNCH --> DORCH[legacy_dig_planner_orchestrator]

    MBE -.订阅.-> PCTOPIC[/material/point_cloud]
    DISP -.调用.-> MTP
    MTP -.调用.-> MBE
    DISP -.action.-> WACT
    DISP -.action.-> DACT
    WACT -.后端.-> AW
    DACT -.后端.-> DORCH
    DORCH -.驱动.-> TRAJ
    DORCH -.驱动.-> LOAD
    DORCH -.驱动.-> RET
```

## 代码结构

- `src/mission_dispatcher`
  - 上层任务调度和状态机
- `src/mobility_planner_core`
  - 行走 action 适配、停靠点规划、边界提取
- `src/excavation_planner_core`
  - 挖掘 action 适配、dig session 编排
- `src/plc_adapter`
  - PLC real/mock 输入
- `src/mission_operator_hmi`
  - 统一上位机
- `src/vendor`
  - 迁入的 legacy 源码
- `docs`
  - 架构、接口、运行、阶段记录

## 快速开始

构建：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_phase2_minimal.sh
```

启动主链：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_phase2/setup.bash
ros2 launch mission_bringup phase2_real.launch.py
```

启动统一上位机：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_hmi_iter/setup.bash
ros2 launch mission_bringup phase2_real.launch.py launch_operator_hmi:=true
```

提交 demo 任务：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_phase2/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```

## 文档入口

- [架构说明](./docs/ARCHITECTURE.md)
- [接口说明](./docs/INTERFACES.md)
- [运行手册](./docs/RUNBOOK.md)
- [系统综述与启动图](./docs/SYSTEM_OVERVIEW_AND_STARTUP.md)
- [阶段变更记录](./docs/CHANGELOG_PHASED.md)
- [开发分支与提交流程](./docs/DEVELOPMENT_WORKFLOW.md)

## Git 分支约定

- `master`
  - 当前稳定主线
- `workspace/...`
  - 日常开发分支
- `backup/...`
  - 关键快照分支，只做留档

当前推荐开发工作区：

- `/home/ruhanguo/shovel_robot/whole_planner_v1_dev_workspace`

## 日常开发

进入开发工作区：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1_dev_workspace
git status
```

提交并推送：

```bash
git add -A
git commit -m "feat: your change"
git push
```

需要把开发分支合回主线时：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
git pull
git merge workspace/20260313-integrated-mission-planner-dev
git push
```
