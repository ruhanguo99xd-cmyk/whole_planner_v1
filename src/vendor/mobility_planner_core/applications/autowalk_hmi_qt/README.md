# autowalk_hmi_qt (Phase A)

Qt 上位机界面（替代 RViz 的第一阶段），接入 `anew_autowalk_v3`。

## 已实现功能（Phase A+）
- 分层显示（可开关）：
  - `/map`
  - `/global_costmap/costmap`
  - `/local_costmap/costmap`
- 显示 URDF 机器人模型（`/robot_description`）
- 显示规划路径（`/plan`）
- 显示手动目标（`/goal_pose`）与自动目标（`/auto_goal_pose`）
- 手动输入目标点并发布（`/goal_pose` 或 `/autonomous_walk/goal_pose`）
- 地图点选目标工具（SetGoal）
- 基础状态栏：里程计位置与速度（`/ekf_odom`）
- 机型参数下拉选择并应用（调用 `scripts/select_machine_profile.py`）

## 编译
```bash
cd /home/ruhanguo/anew_autowalk_v3
colcon build --packages-select autowalk_hmi_qt
source install/setup.bash
```

## 运行
```bash
ros2 launch autowalk_hmi_qt hmi.launch.py
```

## 说明
- 这是 Phase A 的 MVP，优先保证可展示和可交互。
- 后续可扩展：
  - 点击地图设目标（2D Nav Goal）
  - 自动目标点显示（算法目标）
  - 状态栏（模式、速度、定位质量、PLC状态）
  - 多机型参数联动 UI
