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
    LAUNCH[integrated.launch.py] --> PLC[plc_adapter]
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

## 大规划状态机图

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> WALK_PREP: submit_mission + machine_ready + safe_to_walk
    IDLE --> TRANSITION: submit_mission but walk_not_ready
    IDLE --> FAULT: plc_fault/manual_override

    TRANSITION --> WALK_PREP: walk_ready
    TRANSITION --> DIG_PREP: dig_ready
    TRANSITION --> FAULT: plc_fault/manual_override

    WALK_PREP --> WALKING: walk START ack(received)
    WALK_PREP --> FAULT: walk_start_fail / plc_fault

    WALKING --> TRANSITION: /mobility/status=completed and STOP ack
    WALKING --> IDLE: walk canceled
    WALKING --> FAULT: walk failed / plc_fault

    DIG_PREP --> DIGGING: dig START ack(received)
    DIG_PREP --> FAULT: dig_start_fail / plc_fault

    DIGGING --> IDLE: /excavation/status=completed and STOP ack
    DIGGING --> IDLE: dig canceled
    DIGGING --> FAULT: dig failed / plc_fault

    FAULT --> IDLE: recover
```

## 点云到停靠点的数据流图

```mermaid
flowchart LR
    PC[PointCloud2] --> EXTRACTOR[material_boundary_extractor]

    CFG1[boundary_input 配置\nz/roi/angular_bins/min_boundary_radius] --> EXTRACTOR
    CFG2[静态外参\nxyz+rpy] --> EXTRACTOR

    EXTRACTOR --> CHECK{cloud frame\n是否等于目标 frame?}
    CHECK -- 是 --> FILTER[Z过滤 + ROI裁剪]
    CHECK -- 否且有静态外参 --> TF[静态外参变换]
    TF --> FILTER
    CHECK -- 否且无外参 --> FAIL[直接失败]

    FILTER --> BINS[角度分桶/边界抽取]
    BINS --> OUTLINE[/mobility/extract_material_boundary\nboundary_outline]

    OUTLINE --> MTP[material_target_planner]
    MTP --> INPUT[geometry_input / resolved_outline]
    INPUT --> BF[boundary_fit]
    BF --> WB[work_band]
    WB --> CE[candidate_evaluation]
    CE --> TARGET[resolved_walk_target]

    TARGET --> DISP[mission_dispatcher]
    DISP --> WALK[/mobility/execute START]
```

## 当前验证结果

最近一轮仓库内验证记录：

- `scripts/build_workspace.sh`：通过
- HMI 增量构建：`12 packages finished`
- `mission_operator_hmi` 单测：`9 tests, 0 errors, 0 failures`
- GUI 启动探针：通过
- `integrated.launch.py -s` 启动解析：通过

推荐自查命令：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_workspace.sh
colcon --log-base log_install_fix test --build-base build --install-base install --packages-select mission_operator_hmi
colcon --log-base log_install_fix test-result --test-result-base build --all --verbose
bash -lc 'source install/setup.bash'
ros2 launch mission_bringup integrated.launch.py -s
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

标准构建：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_workspace.sh
```

支持的最终运行版本不超过 3 个：

- `mock`
  - 最小闭环验证
- `integrated`
  - 完整联调主链
- `hmi`
  - 单独启动统一上位机

运行 `mock`：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh mock
```

运行 `integrated`：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh integrated
```

运行 `hmi`：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh hmi
```

如果要在完整联调主链里同时带上上位机：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh integrated launch_operator_hmi:=true
```

提交 demo 任务：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```

## 文档入口

- [架构说明](./docs/ARCHITECTURE.md)
- [接口说明](./docs/INTERFACES.md)
- [运行手册](./docs/RUNBOOK.md)
- [系统综述与启动图](./docs/SYSTEM_OVERVIEW_AND_STARTUP.md)
- [阶段变更记录](./docs/CHANGELOG_PHASED.md)
- [开发分支与提交流程](./docs/DEVELOPMENT_WORKFLOW.md)

## Roadmap / TODO

### P0 稳定化

- [ ] 用真实传感器外参与真实点云，再跑一轮 `PointCloud2 -> boundary -> target_pose` 在线验收
- [ ] 继续补 dig cancel 在更底层后处理链的停止检查点，避免未来扩展后出现“上层停了，底层尾段还在算”
- [ ] 把 dispatcher 当前较粗的 `TRANSITION` 状态继续拆细成更清晰的等待态和切换态
- [ ] 把统一上位机纳入更完整的回归验收，而不只做单包 GUI 自测

### P1 算法增强

- [ ] 把真正的物料点云分割/边界提取前处理接入 `material_boundary_extractor`
- [ ] 把停靠点规划继续按专利思路细化，增强对不同物料形态和边界退化场景的鲁棒性
- [ ] 给 dig 结束后“下一次应该往哪里走”补正式接口，不再只回到 `IDLE` 等新目标
- [ ] 继续把物料类型、机型参数、候选点评价权重做成更完整的参数体系

### P2 上位机与交付

- [ ] 给 GitHub 首页补统一上位机截图和实际运行效果图
- [ ] 继续增强挖掘页优化过程可视化，让调试信息更容易用于联调和演示
- [ ] 把更多测试并入统一 `colcon test` 回归链
- [ ] 补齐更标准的发布/部署说明，让新机器落地更顺
- [ ] 按阶段整理更干净的 feature 分支和 release 里程碑

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
