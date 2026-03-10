# RUNBOOK

## 环境准备

1. ROS 2 Humble
2. `colcon`
3. `nav2_msgs` 可用
4. `python3-yaml` 可用
5. 如果要启用真实 PLC：
   - 安装 `snap7`
   - 配置 `config/plc/real_plc.example.yaml`

## Phase 1：mock 最小闭环

### 构建
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
colcon build --packages-select integrated_mission_interfaces mission_dispatcher plc_adapter mobility_planner_core excavation_planner_core mission_bringup
source install/setup.bash
```

### 启动
```bash
ros2 launch mission_bringup phase1_mock.launch.py
```

### 提交 demo 任务
```bash
ros2 run mission_dispatcher submit_demo_mission
```

### 期望日志
- `IDLE -> WALK_PREP -> WALKING -> TRANSITION -> DIG_PREP -> DIGGING -> IDLE`

## Phase 2：最小真实链

### 构建
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_phase2_minimal.sh
```

### 启动
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_phase2_real.sh
```

### 提交 demo 任务
```bash
source install_phase2/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```

### 当前最小真实链组成
- 真实 walk 核心：`autonomous_walk`
- walk 环境 mock：`mock_nav2_server`
- 真实 dig 动作逻辑：`plc_control_test1`
- dig 感知兼容 mock：`legacy_perception_notifier`

## Phase 3：PLC real/mock 切换

### mock
```bash
ros2 run plc_adapter plc_adapter_node --ros-args -p backend:=mock -p mock_sequence_file:=/home/ruhanguo/shovel_robot/whole_planner_v1/config/plc/mock_sequence.yaml
```

### real
```bash
ros2 run plc_adapter plc_adapter_node --ros-args --params-file /home/ruhanguo/shovel_robot/whole_planner_v1/config/plc/real_plc.example.yaml
```

## 常见故障

### `not found ... tra_planning/local_setup.bash`
- 原因：历史失败构建残留在默认 `install/`
- 处理：优先使用 `install_phase2`，或手工清理旧 `build/install/log` 后重建

### `nlopt` 缺失导致 `tra_planning` 不能编译
- 当前默认策略：源码保留，`COLCON_IGNORE` 关闭默认构建
- 处理：安装 `nlopt` 开发库后移除 `src/vendor/excavation_planner_core/tra_planning/COLCON_IGNORE`

### legacy action 出现 `unexpected goal/result response`
- 当前现象：最小真实链能完成，但 ROS2 Python action 在 legacy 组合下有重复响应告警
- 影响：当前闭环可跑，日志会有噪音
- 后续：建议把 dig legacy 适配层从 Python action client 收敛到单线程串行桥接节点
