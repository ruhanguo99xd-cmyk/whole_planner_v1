# anew_autowalk_v3 (Pruned)

精简版工程，保留真实运行主链：

- `bringup/launch/bringup.launch.py`
- `bringup/launch/ekf_bringup.launch.py`

以及保留包（含你明确要求保留的运行/仿真关键包）：

- `src/description`
- `src/recorders/path_to_csv`
- `src/recorders/cmd_vel_recorder`
- `src/navigation/map_server`
- `src/localization/rtk_to_odom`
- `src/control/cmd_vel_to_plc`
- `src/perception/lite_slam`
- `src/localization/fake_odom`（仿真必需，已恢复）
- `src/control/init_pose`（切换行走模式固定姿态，已恢复）
- `src/applications/autonomous_walk`（上层任务接口，已恢复）
- `src/applications/planner_client`（路径服务接口，已恢复）
- `src/applications/autowalk_hmi_qt`（Qt上位机，Phase A）

## 运行

```bash
cd /home/ruhanguo/anew_autowalk_v3
colcon build
source install/setup.bash
ros2 launch bringup ekf_bringup.launch.py use_lite_slam:=true
```

## autonomous_walk 调用规范

详见：
- `docs/INTERFACE_SPEC_AUTONOMOUS_WALK.md`

适合大规划/调度系统统一调用 `/autonomous_walk/set_goal`。

## 机型参数管理（7机型）

详见：
- `docs/MACHINE_PROFILE_GUIDE.md`
- `config/machines/README.md`

切换机型：
```bash
python3 scripts/select_machine_profile.py --machine M003
```

或者直接启动时指定：
```bash
ros2 launch bringup ekf_bringup.launch.py machine_profile:=M003 use_lite_slam:=true
```

会生成/使用：`config/machine.active.yaml`，并在 launch 中自动加载对应机型参数。

## 说明

- 本版本已移除部分非主链内容并归档到：
  - `_pruned_backup_YYYYMMDD-HHMMSS/`
- 当前已按现场要求恢复 `fake_odom`、`init_pose`、`autonomous_walk`、`planner_client`。
