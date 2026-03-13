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

## 最终支持的运行版本

当前建议只保留 3 个正式运行版本：

1. `mock`
   - 最小闭环验证
2. `integrated`
   - 完整联调主链
3. `hmi`
   - 单独启动统一上位机

统一入口脚本：

```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_workspace.sh
bash scripts/run_profile.sh mock
bash scripts/run_profile.sh integrated
bash scripts/run_profile.sh hmi
```

说明：
- `scripts/build_phase2_minimal.sh`
- `scripts/run_phase1_mock.sh`
- `scripts/run_phase2_real.sh`

这 3 个旧脚本仍然保留，但现在只作为兼容包装，内部会转发到新的标准入口。

文档里如果看到 `install_phase2`、`install_pointcloud_ci` 之类路径，那是历史验证记录保留下来的旧前缀。
当前日常使用请优先统一到：

- `install/setup.bash`
- `bash scripts/build_workspace.sh`
- `bash scripts/run_profile.sh <mock|integrated|hmi>`

## 运行版本 1：mock 最小闭环

### 构建
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_workspace.sh
```

### 启动
```bash
bash scripts/run_profile.sh mock
```

### 提交 demo 任务
```bash
source install/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```

### 期望日志
- `IDLE -> WALK_PREP -> WALKING -> TRANSITION -> DIG_PREP -> DIGGING -> IDLE`

## 运行版本 2：integrated 完整联调主链

### 构建
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/build_workspace.sh
```

`build_workspace.sh` 会先检查系统里是否已安装 `libnlopt-dev`，缺失时直接退出。

### 启动
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh integrated
```

### 提交 demo 任务
```bash
source install/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```

### 运行版本 3：hmi 单独上位机
单独启动：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh hmi
```

### integrated 模式里同时带上上位机
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
bash scripts/run_profile.sh integrated launch_operator_hmi:=true
```

统一上位机当前提供：
- `大规划` 页签：开始/停止/恢复、手动任务表单、示例任务提交、状态事件流
- `行走规划` 页签：参考 `autowalk_hmi_qt`，接入 `map/global_costmap/local_costmap`、目标点、自动目标、物料边界、规划路径、状态灯、`cmd_vel` 与履带速度曲线，并支持地图点选目标
- `挖掘规划` 页签：dig 阶段、回转角、`trajectory_planner` 三段实时轨迹、优化中间候选轨迹、推压/提升速度曲线、`load/return` 回转曲线

### 统一上位机交互提示

- 地图点选目标：
  - 进入 `行走规划` 页签
  - 勾选“点选目标模式”
  - 在地图上按下设定目标位置
  - 拖动设定朝向
  - 松开后自动发布到 `/goal_pose`
- 挖掘实时轨迹：
  - `Seg1/Seg2/Seg3` 是本轮规划的最终三段轨迹
  - `Opt Candidate` 是优化过程中正在尝试的候选轨迹
  - `优化中间态` 卡片显示当前 `eval/objective/fill/tf`

### 当前最小真实链组成
- 真实 walk 核心：`autonomous_walk`
- walk 环境 mock：`mock_nav2_server`
- 停靠点接口：`material_target_planner`
- 在线点云边界提取：`material_boundary_extractor`
- dig 调度桥：`legacy_dig_planner_orchestrator`
- dig 真实规划链：`prsdata_server`、`perceive_truck_server`、`trajectory_planner`、`load`、`return`
- action 协议：`START/STOP/CANCEL` 命令 + `/mobility/status`、`/excavation/status` 状态回传
- 默认启动模式：`launch_legacy_dig_planner:=true`
- 回退旧链路：
```bash
ros2 launch mission_bringup phase2_real.launch.py launch_legacy_dig:=true launch_legacy_dig_planner:=false
```
- 停靠点规划内部结构：
  - `boundary_fit`
  - `work_band`
  - `candidate_evaluation`
- `boundary_fit` 当前实现要点：
  - 支持 request `material_outline`、`boundary_input.outline_points`、`boundary_input.line_strips`、`boundary_input.scatter_points`
  - 支持 `/mobility/extract_material_boundary` 在线点云边界提取
  - `scatter_points` 会先做 ROI 裁剪和角度分桶，再提取边界点
  - 边界点去重与栅格滤波
  - 弧长排序与边界曲线采样
  - 最近边缘、法向、曲率、拟合误差输出
- `work_band` 当前实现要点：
  - `min/preferred/max` 三层偏置边界
  - 基于接近距离、朝向变化、障碍净空、局部坡度、估计转弯半径的站位可行性裁剪
  - 最大连通可行站位段保留
- `candidate_evaluation` 当前实现要点：
  - 路径、参考点、偏置距离代价
  - 安全代价：障碍、坡度、转弯半径
  - 曲率和朝向代价

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
   - `Mission queued: ... walk_resolution=material_target:...`
   - `Sent start command to walk`
   - `Sent stop command to walk`
   - `WALKING -> TRANSITION`
   - `TRANSITION -> DIG_PREP -> DIGGING`
    - `Sent start command to dig`
   - `trajectory_planner` 发布 `excavation_end_length`
   - `load_node` 输出 `csv_created/load`
   - `return_node` 输出 `csv_created/return`
   - `Sent stop command to dig`
   - `DIGGING -> IDLE`
7. 抽查 status topic
```bash
source install_phase2/setup.bash
timeout 8s ros2 topic echo /mobility/status --once
timeout 8s ros2 topic echo /excavation/status --once
```
8. 抽查挖掘实时调试 topic
```bash
source install_phase2/setup.bash
timeout 8s ros2 topic echo /digging/debug/segment3_path --once
timeout 8s ros2 topic echo /digging/debug/optimization_candidate_path --once
timeout 8s ros2 topic echo /digging/debug/optimization_metrics --once
timeout 8s ros2 topic echo /digging/debug/load_rotation_deg --once
```
9. 抽查直接 action 协议
```bash
source install_phase2/setup.bash
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 1, mission_id: 'check-walk', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 2, mission_id: 'check-walk', target_pose: {header: {frame_id: 'map'}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 3, mission_id: 'check-walk', target_pose: {header: {frame_id: 'map'}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 1, mission_id: 'check-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 2, mission_id: 'check-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 3, mission_id: 'check-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
```
10. 期望 action result
   - `accepted: true`
   - `message: received`

### dig cancel 在线验收

1. 启动真实联调链，适当降低 mock 回转速度，给 cancel 留窗口
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_phase2/setup.bash
ros2 launch mission_bringup phase2_real.launch.py mock_swing_speed_deg_per_sec:=10.0
```
2. 另一个终端提交任务
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_phase2/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```
3. 看到 `DIG_PREP -> DIGGING` 后，触发 stop
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_phase2/setup.bash
ros2 service call /mission_dispatcher/stop std_srvs/srv/Trigger '{}'
```
4. 期望日志
   - `Sent cancel command to dig`
   - `Dig planner status -> cancel_requested`
   - `Dig planner status -> canceled`
   - `trajectory_planner 已取消：optimization_running`
   - `load 在等待开始信号阶段被取消`
   - `return 在等待开始信号阶段被取消`
5. 验收要点
   - cancel 后不应再看到新的 `load/return 规划完成` 日志
   - cancel 后工作区根目录不应新生成 `csv_created/load`、`csv_created/return`

### 通过物料接口提交任务
```bash
source install_phase2/setup.bash
ros2 run mission_dispatcher submit_demo_mission
```

当前 demo 会走：
- `use_material_target=true`
- `mission_dispatcher -> /mobility/compute_material_target -> resolved_walk_target`
- `resolved_walk_target -> /mobility/execute`
- `debug_json.layers == ["boundary_fit", "work_band", "candidate_evaluation"]`
- `debug_json.planner == "material_target_planner_v4"`
- `debug_json.work_band.kept_station_count > 0`

### 单独验证 material target 规划层
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_regression_ci/setup.bash
python3 -m pytest -q src/mobility_planner_core/test/test_material_target_planner.py
```

当前覆盖点：
- 退化直线边界拟合
- 闭合边界曲线长度与最近弧长
- `boundary_input.outline_points`
- `boundary_input.line_strips`
- `boundary_input.scatter_points`
- `PointCloud2 -> boundary extractor -> outline`
- `edge_near` / `far_field` 两类策略
- `S410` 的偏置边界与可行站位段裁剪
- `S414/S415` 的障碍净空、坡度、转弯半径裁剪
- `S510` 的多目标候选点评价

### 统一上位机最小验收
1. 构建并安装：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
colcon --log-base log_pointcloud_ci build \
  --build-base build_pointcloud_ci \
  --install-base install_pointcloud_ci \
  --packages-select mission_operator_hmi mission_bringup
```
2. 运行单测：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
colcon --log-base log_pointcloud_ci test \
  --build-base build_pointcloud_ci \
  --install-base install_pointcloud_ci \
  --packages-select mission_operator_hmi
```
3. 启动上位机：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_pointcloud_ci/setup.bash
ros2 run mission_operator_hmi integrated_operator_hmi
```
4. 期望行为：
   - 可以在一个窗口里切换 `大规划 / 行走规划 / 挖掘规划`
   - `大规划` 页签能看到 `/mission_dispatcher/mode`、`/mobility/status`、`/excavation/status`、`/plc/status`
   - `行走规划` 页签能显示 `map/global_costmap/local_costmap`、`/plan`、当前位置、目标点和履带速度曲线
   - `行走规划` 页签开启“点选目标模式”后，可以鼠标拖拽下发 `/goal_pose`
   - `挖掘规划` 页签能直接显示 `digging/debug/*` 实时轨迹、优化候选轨迹和平滑回转曲线

当前直接 `pytest` 口径：
- `23 passed`

当前干净 `colcon test` 口径：
- `mobility_planner_core`: `24 tests`
- `mission_dispatcher + plc_adapter + mobility_planner_core + excavation_planner_core`: `36 tests`

### 启用在线点云边界提取
默认 `phase2_real.launch.py` 已包含：
- `material_boundary_extractor`
- `material_target_planner(enable_boundary_extractor=true)`
- `config/perception/material_boundary_extrinsic/<machine_model>.yaml` 自动加载

静态外参配置目录：
- `config/perception/material_boundary_extrinsic/default.yaml`
- `config/perception/material_boundary_extrinsic/prototype.yaml`

当前加载规则：
- 优先读取 `config/perception/material_boundary_extrinsic/<machine_model>.yaml`
- 如果该机型文件不存在，回退到 `config/perception/material_boundary_extrinsic/default.yaml`
- 命令行传入的 `boundary_*` launch 参数仍然会覆盖 YAML 中的同名参数

如需指定真实点云 topic：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_pointcloud_ci/setup.bash
ros2 launch mission_bringup phase2_real.launch.py material_point_cloud_topic:=/your/point_cloud_topic
```

如果你的点云 frame 和 `current_pose/material_reference_pose` 不是同一个坐标系，推荐先把固定外参写进对应机型 YAML，然后按机型启动：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_pointcloud_ci/setup.bash
ros2 launch mission_bringup phase2_real.launch.py machine_model:=prototype material_point_cloud_topic:=/your/point_cloud_topic
```

如需临时覆盖 YAML 中的外参，也可以继续在命令行显式传入：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_pointcloud_ci/setup.bash
ros2 launch mission_bringup phase2_real.launch.py \
  material_point_cloud_topic:=/your/point_cloud_topic \
  use_boundary_static_extrinsic:=true \
  boundary_sensor_frame_id:=your_sensor_frame \
  boundary_extrinsic_translation_x_m:=1.2 \
  boundary_extrinsic_translation_y_m:=0.0 \
  boundary_extrinsic_translation_z_m:=2.4 \
  boundary_extrinsic_roll_deg:=0.0 \
  boundary_extrinsic_pitch_deg:=0.0 \
  boundary_extrinsic_yaw_deg:=90.0
```

说明：
- 这组外参表示“传感器坐标系到目标规划坐标系”的静态位姿
- 当前没有 `tf` 树时，这是必填项
- 如果 frame 不一致又没配外参，提取器会直接报错，不会继续算

### 通过点云边界提取链路提交停靠点计算
在 `target_planner_constraints_json` 中指定：
```json
{
  "boundary_input": {
    "source": "pointcloud",
    "z_min_m": -0.5,
    "z_max_m": 1.0,
    "roi_center": "material_reference_pose",
    "roi_radius_m": 8.0,
    "angular_bins": 32,
    "min_boundary_radius_m": 0.5,
    "min_points_required": 8,
    "static_extrinsic": {
      "enabled": true,
      "sensor_frame_id": "hesai_frame",
      "translation_x_m": 1.2,
      "translation_y_m": 0.0,
      "translation_z_m": 2.4,
      "roll_deg": 0.0,
      "pitch_deg": 0.0,
      "yaw_deg": 90.0
    }
  }
}
```

期望行为：
- `material_target_planner` 调用 `/mobility/extract_material_boundary`
- `material_boundary_extractor` 从最新 `PointCloud2` 提取 `boundary_outline`
- 如果 frame 不一致，则先应用静态外参
- `debug_json.geometry_input.source == "pointcloud_boundary_extractor"`
- `debug_json.pointcloud_boundary_extractor.status == "ok"`

### 在线点云边界链快速探针
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_pointcloud_ci/setup.bash
ros2 run mobility_planner_core material_boundary_extractor
```

另一个终端：
```bash
cd /home/ruhanguo/shovel_robot/whole_planner_v1
source install_pointcloud_ci/setup.bash
ros2 run mobility_planner_core material_target_planner --ros-args -p enable_boundary_extractor:=true
```

再另一个终端发布合成点云并调用停靠点服务后，期望看到：
- `success=True`
- `geometry_source=pointcloud_boundary_extractor`
- `extractor_status=ok`
- `fit_method=piecewise_cubic_boundary`

### 通过约束 JSON 注入障碍、坡度和权重
```json
{
  "boundary_input": {
    "source": "scatter_points",
    "scatter_points": [
      {"x": -2.0, "y": 0.0},
      {"x": -1.4, "y": -1.4},
      {"x": 0.0, "y": -2.0},
      {"x": 1.4, "y": -1.4},
      {"x": 2.0, "y": 0.0},
      {"x": 1.4, "y": 1.4},
      {"x": 0.0, "y": 2.0},
      {"x": -1.4, "y": 1.4}
    ],
    "angular_bins": 16,
    "roi_center": "material_reference_pose",
    "roi_radius_m": 8.0,
    "min_boundary_radius_m": 0.5
  },
  "work_band": {
    "preferred_offset_m": 2.8,
    "min_turn_radius_m": 3.0,
    "obstacle_clearance_m": 0.8,
    "max_slope_deg": 9.0,
    "obstacles": [
      {"x": -3.8, "y": 0.0, "radius_m": 0.6}
    ],
    "slope_zones": [
      {"x": -3.8, "y": 0.0, "radius_m": 1.0, "slope_deg": 6.0}
    ]
  },
  "candidate_evaluation": {
    "path_weight": 1.0,
    "reference_weight": 0.45,
    "distance_weight": 0.9,
    "safety_weight": 1.4,
    "curvature_weight": 0.5,
    "yaw_weight": 0.2
  }
}
```

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

### dig cancel 后底层还在继续规划
- 现象：dispatcher 已切回 `IDLE`，但 `trajectory_planner`、`load`、`return` 仍继续输出本轮完成日志
- 检查：
  - `legacy_dig_planner_orchestrator` 是否发布了 `digging/cancel`
  - `trajectory_planner` 是否出现 `收到 dig cancel` 和 `Optimization canceled!`
  - `load_node`、`return_node` 是否出现 `收到 dig cancel`
- 处理：
```bash
source install_phase2/setup.bash
ros2 topic echo /excavation/status
ros2 topic echo /lite_slam/swing_angle_deg
```
  - 优先确认当前启动入口是 `launch_legacy_dig_planner:=true`

## 常见操作

### 直接向 walk 发送开始/停止命令
```bash
source install_phase2/setup.bash
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 1, mission_id: 'manual-walk', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 2, mission_id: 'manual-walk', target_pose: {header: {frame_id: 'map'}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
ros2 action send_goal /mobility/execute integrated_mission_interfaces/action/WalkMission "{command: 3, mission_id: 'manual-walk', target_pose: {header: {frame_id: 'map'}}, constraints_json: '{}', priority: 1, timeout_sec: 5.0}"
```

### 直接向 dig 发送开始/停止命令
```bash
source install_phase2/setup.bash
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 1, mission_id: 'manual-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 2, mission_id: 'manual-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
ros2 action send_goal /excavation/execute integrated_mission_interfaces/action/DigMission "{command: 3, mission_id: 'manual-dig', target_zone: 'bench-A', process_parameters_json: '{}', safety_boundary_json: '{}', priority: 1, timeout_sec: 10.0}"
```

### 查看大规划状态
```bash
source install/setup.bash
ros2 topic echo /mission_dispatcher/mode
```
