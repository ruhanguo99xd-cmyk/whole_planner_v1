# RUNBOOK

## 环境准备

1. ROS 2 Humble
2. `colcon`
3. `libnlopt-dev`
4. `nav2_msgs` 可用
5. `python3-yaml` 可用
6. 如果要启用真实 PLC：
   - 安装 `snap7`
   - 配置 `config/plc/real_plc.example.yaml`

### Ubuntu 依赖安装
```bash
sudo apt-get update
sudo apt-get install -y libnlopt-dev
```

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

`build_phase2_minimal.sh` 会先检查系统里是否已安装 `libnlopt-dev`，缺失时直接退出。

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
- action 协议：`START/STOP` 命令 + `/mobility/status`、`/excavation/status` 状态回传

### 系统库安装后的验收 Checklist

1. 检查系统库
```bash
dpkg -s libnlopt-dev
pkg-config --modversion nlopt
```
2. 全量重编译
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_phase2_minimal.sh
```
3. 跑单测
```bash
colcon --log-base log_phase2 test --build-base build_phase2 --install-base install_phase2 --packages-select mission_dispatcher plc_adapter mobility_planner_core excavation_planner_core
colcon --log-base log_phase2 test-result --all --verbose
```
4. 启动真实联调链
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_phase2_real.sh
```
5. 提交大规划 demo 任务
```bash
source install_phase2/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```
6. 期望日志
   - `IDLE -> WALK_PREP -> WALKING`
   - `Sent start command to walk`
   - `Sent stop command to walk`
   - `WALKING -> TRANSITION`
   - `TRANSITION -> DIG_PREP -> DIGGING`
   - `Sent start command to dig`
   - `Sent stop command to dig`
   - `DIGGING -> IDLE`
7. 抽查 status topic
```bash
source install_phase2/setup.bash
timeout 8s ros2 topic echo /mobility/status --once
timeout 8s ros2 topic echo /excavation/status --once
```
8. 抽查直接 action 协议
```bash
source install_phase2/setup.bash
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 1, mission_id: 'check-walk', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 2, mission_id: 'check-walk', target_pose: {header: {frame_id: 'map'}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 1, mission_id: 'check-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 2, mission_id: 'check-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
```
9. 期望 action result
   - `accepted: true`
   - `message: received`

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
- 现象：`build_phase2_minimal.sh` 直接报缺少 `libnlopt-dev`
- 处理：
```bash
sudo apt-get update
sudo apt-get install -y libnlopt-dev
```

### 子系统已经完成但大规划不切相
- 现象：walk/dig 已完成，但 dispatcher 还停在 `WALKING` 或 `DIGGING`
- 检查：
  - `/mobility/status` 或 `/excavation/status` 是否发布了 `completed`
  - 随后是否发布了 `stopped` 或 `idle`
  - `mission_id` 是否与当前任务一致
- 处理：优先排查 legacy backend 的状态发布链，不要在 dispatcher 内硬编码补状态

## 常见操作

### 直接向 walk 发送开始/停止命令
```bash
source install_phase2/setup.bash
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 1, mission_id: 'manual-walk', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 2, mission_id: 'manual-walk', target_pose: {header: {frame_id: 'map'}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
```

### 直接向 dig 发送开始/停止命令
```bash
source install_phase2/setup.bash
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 1, mission_id: 'manual-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 2, mission_id: 'manual-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
```

### 查看大规划状态
```bash
source install/setup.bash
ros2 topic echo /mission_dispatcher/mode
```
