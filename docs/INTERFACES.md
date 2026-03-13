# INTERFACES

## 调度服务

### `/mission_dispatcher/submit_mission`
- 类型：`integrated_mission_interfaces/srv/SubmitMission`
- 输入：
  - `mission_id`
  - `use_material_target`
  - `current_pose`
  - `walk_target`
  - `material_reference_pose`
  - `material_outline`
  - `desired_standoff_m`
  - `material_profile_json`
  - `target_planner_constraints_json`
  - `dig_target_zone`
  - `walk_constraints_json`
  - `dig_parameters_json`
  - `priority`
- 输出：
  - `accepted`
  - `message`
  - `resolved_walk_target`
  - `resolution_detail`

说明：
- `use_material_target=false`：沿用外部直接给定的 `walk_target`
- `use_material_target=true`：`mission_dispatcher` 先调用 `/mobility/compute_material_target`，再把解析后的 `resolved_walk_target` 下发给 walk

### `/mobility/compute_material_target`
- 类型：`integrated_mission_interfaces/srv/ComputeMaterialTarget`
- 作用：根据物料几何和约束生成停靠目标点接口骨架
- 当前实现：
  - 三层结构：
    - `boundary_fit`：投影边界点、栅格滤波、弧长排序、边界曲线采样、法向/曲率/拟合质量计算
    - `work_band`：沿边界法向生成内/优选/外三层偏置边界，并按可达性约束裁剪可行站位段
    - `candidate_evaluation`：按路径、参考点、偏置距离、安全性、曲率和朝向代价选最终停靠点
  - 上游几何输入适配：
    - 优先直接使用 request `material_outline`
    - 也支持从 `boundary_input` / `geometry_input` JSON 中解析：
      - `outline_points`
      - `line_strips`
      - `scatter_points`
    - `scatter_points` 会先做 ROI 裁剪、点数限制和角度分桶，再提取边界输入给 `boundary_fit`
    - 也支持通过 `/mobility/extract_material_boundary` 从最新 `PointCloud2` 在线提取边界，再回填给停靠点规划
  - `edge_near`：当前车体靠近物料边缘时，按局部边缘外法向退让
  - `far_field`：当前车体位于物料远端时，从可行作业带中选取当前接近方向最优的站位段
- 当前输出：
  - `target_pose`
  - `walk_constraints_json`
  - `debug_json`

`debug_json` 当前主要字段：
- `boundary_fit`
- `geometry_input`
- `work_band`
- `candidate_evaluation`
- `layers`
- `profile_keys`
- `constraint_keys`
- `boundary_fit_config`
- `work_band_config`
- `candidate_weight_config`

`geometry_input` 当前输出的关键指标：
- `source`
- `boundary_mode`
- `preferred_source`
- `request_outline_count`
- `json_outline_count`
- `line_strip_point_count`
- `scatter_point_count`
- `selected_input_count`
- `roi_filtered_count`
- `outline_point_count`
- `roi_radius_m`
- `roi_center_source`
- `angular_bins`
- `min_boundary_radius_m`
- `max_input_points`
- `centroid_hint_source`

`boundary_input` / `geometry_input` 当前可通过 `material_profile_json` 或 `target_planner_constraints_json` 传入：
- `boundary_input.source`
- `boundary_input.outline_points`
- `boundary_input.line_strips`
- `boundary_input.scatter_points`
- `boundary_input.raw_points`
- `boundary_input.roi_center`
- `boundary_input.roi_radius_m`
- `boundary_input.centroid_hint`
- `boundary_input.angular_bins`
- `boundary_input.max_input_points`
- `boundary_input.min_boundary_radius_m`

说明：
- `outline_points` 适合已提取的边界点
- `line_strips` 适合上游输出的分段轮廓线
- `scatter_points` 适合点云投影后的散点输入

`boundary_fit` 当前输出的关键指标：
- `fit_method`
- `curve_length_m`
- `nearest_edge_distance_m`
- `nearest_arc_length_m`
- `closure_gap_m`
- `mean_fit_error_m`
- `signed_area`
- `point_count_input`
- `point_count_filtered`
- `point_count_samples`

`boundary_fit_config` 当前可通过 `material_profile_json` 或 `target_planner_constraints_json` 传入：
- `boundary_fit.grid_size_m`
- `boundary_fit.min_points_per_cell`
- `boundary_fit.sample_count`

`work_band` 当前输出的关键指标：
- `min_offset_m`
- `preferred_offset_m`
- `max_offset_m`
- `station_count`
- `kept_station_count`
- `candidate_count`
- `min_turn_radius_m`
- `obstacle_clearance_m`
- `max_slope_deg`

`work_band_config` 当前可通过 `material_profile_json` 或 `target_planner_constraints_json` 传入：
- `work_band.min_offset_m`
- `work_band.preferred_offset_m`
- `work_band.max_offset_m`
- `work_band.max_candidates`
- `work_band.min_approach_distance_m`
- `work_band.max_approach_distance_m`
- `work_band.max_heading_change_deg`
- `work_band.max_boundary_curvature`
- `work_band.min_turn_radius_m`
- `work_band.obstacle_clearance_m`
- `work_band.max_slope_deg`
- `work_band.preferred_yaw_offset_deg`
- `work_band.obstacles`
- `work_band.slope_zones`

`candidate_weight_config` 当前可通过 `material_profile_json` 或 `target_planner_constraints_json` 传入：
- `candidate_evaluation.path_weight`
- `candidate_evaluation.reference_weight`
- `candidate_evaluation.distance_weight`
- `candidate_evaluation.safety_weight`
- `candidate_evaluation.curvature_weight`
- `candidate_evaluation.yaw_weight`

兼容说明：
- 旧的顶层字段 `path_weight`、`reference_weight`、`yaw_weight` 仍然可用
- 新增的多目标权重优先从 `candidate_evaluation` 读取

### `/mobility/extract_material_boundary`
- 类型：`integrated_mission_interfaces/srv/ExtractMaterialBoundary`
- 作用：从最新 `PointCloud2` 缓存中提取 2D 物料边界
- 输入：
  - `request_id`
  - `current_pose`
  - `material_reference_pose`
  - `material_profile_json`
  - `planner_constraints_json`
- 输出：
  - `success`
  - `message`
  - `boundary_outline`
  - `debug_json`

当前实现：
- 订阅 `sensor_msgs/msg/PointCloud2`
- 读取最新点云缓存
- 如果 `PointCloud2.frame_id` 与目标规划坐标系不一致：
  - 必须提供静态外参
  - 否则直接报错，不继续做边界提取
- 依据 `boundary_input` / `geometry_input` 配置执行：
  - `z_min_m / z_max_m`
  - `roi_center / roi_radius_m`
  - `angular_bins`
  - `max_input_points`
  - `min_boundary_radius_m`
  - `min_points_required`
  - `static_extrinsic`
- 输出：
  - `boundary_outline`
  - `debug_json.config`
  - `debug_json.frame_transform`
  - `debug_json.scatter_outline`

`boundary_input.static_extrinsic` 当前可通过 `material_profile_json` 或 `target_planner_constraints_json` 传入：
- `boundary_input.static_extrinsic.enabled`
- `boundary_input.static_extrinsic.sensor_frame_id`
- `boundary_input.static_extrinsic.translation_x_m`
- `boundary_input.static_extrinsic.translation_y_m`
- `boundary_input.static_extrinsic.translation_z_m`
- `boundary_input.static_extrinsic.roll_deg`
- `boundary_input.static_extrinsic.pitch_deg`
- `boundary_input.static_extrinsic.yaw_deg`

相关调试 topic：
- `/mobility/material_boundary`
- `/mobility/material_boundary_debug`

### `/mission_dispatcher/start`
- 类型：`std_srvs/srv/Trigger`
- 作用：启用自动调度。

### `/mission_dispatcher/stop`
- 类型：`std_srvs/srv/Trigger`
- 作用：停止自动调度，并向当前子系统发送 `CANCEL` 指令。

### `/mission_dispatcher/recover`
- 类型：`std_srvs/srv/Trigger`
- 作用：FAULT 锁存解除后回到 `IDLE`。

## Command 协议

- 大规划对 walk/dig 发送三类命令：
  - `START=1`
  - `STOP=2`
  - `CANCEL=3`
- action result 只表示命令是否被接收：
  - `accepted=true`
  - `message=received`
- 长时执行状态、进度、故障和完成信号统一从 status topic 回传给 `mission_dispatcher`。

## Walk Action

### 名称
- `/mobility/execute`
- 类型：`integrated_mission_interfaces/action/WalkMission`

### Goal
- `command`
- `mission_id`
- `target_pose`
- `constraints_json`
- `priority`
- `timeout_sec`

### Result
- `accepted`
- `error_code`
- `message`

### Feedback
- `phase`
- `progress`

## Dig Action

### 名称
- `/excavation/execute`
- 类型：`integrated_mission_interfaces/action/DigMission`

### Goal
- `command`
- `mission_id`
- `target_zone`
- `process_parameters_json`
- `safety_boundary_json`
- `priority`
- `timeout_sec`

### Result
- `accepted`
- `error_code`
- `message`

### Feedback
- `phase`
- `progress`

## PLC 输入

### `/plc/status`
- 类型：`integrated_mission_interfaces/msg/PlcSnapshot`
- 字段：
  - `machine_ready`：整机调度允许位
  - `safe_to_walk`：允许切入行走
  - `safe_to_dig`：允许切入挖掘
  - `walking_requested`：PLC 指示当前处于行走作业窗口
  - `digging_requested`：PLC 指示当前处于挖掘作业窗口
  - `fault_active`：PLC 故障锁存
  - `manual_override`：人工接管
  - `fault_code`：PLC 故障码
  - `source`：`mock` 或 `real_plc`

## 状态输出

### `/mission_dispatcher/mode`
- 类型：`integrated_mission_interfaces/msg/PlannerMode`
- 模式枚举：
  - `IDLE`
  - `WALK_PREP`
  - `WALKING`
  - `DIG_PREP`
  - `DIGGING`
  - `TRANSITION`
  - `FAULT`

### `/mobility/status` 和 `/excavation/status`
- 类型：`integrated_mission_interfaces/msg/SubsystemStatus`
- 字段：
  - `mission_id`
  - `active`
  - `ready`
  - `error`
  - `error_code`
  - `phase`
  - `detail`
  - `progress`
  - `stamp`

### 状态语义

- walk 常用 `phase`
  - `idle`
  - `starting`
  - `walking`
  - `completed`
  - `stopping`
  - `stopped`
  - `cancel_requested`
  - `canceled`
  - `failed`
- dig 常用 `phase`
  - `idle`
  - `starting`
  - `rotating_scan`
  - `waiting_perception`
  - `planning_retry`
  - `resetting`
  - `digging`
  - `cycle_complete`
  - `completed`
  - `stopping`
  - `stopped`
  - `cancel_requested`
  - `canceled`
  - `failed`
- dig cancel 下沉链
  - `mission_dispatcher/stop`
  - `/excavation/execute command=CANCEL`
  - `legacy_dig_planner_orchestrator /dig/cancel`
  - `digging/cancel`
  - `trajectory_planner` 中断当前 `nlopt` 优化
  - `load` / `return` 中断等待和本轮计算
- 调度判定规则
  - `completed`：大规划发 `STOP`
  - `stopped` 或 `idle` 且 `active=false`：当前阶段正式收口
  - `canceled` 且 `active=false`：当前阶段已被撤销
  - `error=true`：进入失败处理、重试或 `FAULT`

## 调试可视化 Topic

### 行走页实时订阅

- `/map`
  - 类型：`nav_msgs/msg/OccupancyGrid`
  - 说明：主地图层
- `/global_costmap/costmap`
  - 类型：`nav_msgs/msg/OccupancyGrid`
  - 说明：全局代价地图层
- `/local_costmap/costmap`
  - 类型：`nav_msgs/msg/OccupancyGrid`
  - 说明：局部代价地图层
- `/cmd_vel`
  - 类型：`geometry_msgs/msg/Twist`
  - 说明：统一上位机据此计算左右履带速度曲线
- `/goal_pose`
  - 类型：`geometry_msgs/msg/PoseStamped`
  - 说明：统一上位机手动目标与地图点选目标都发布到这里

行走页交互说明：
- 开启“点选目标模式”后：
  - 鼠标按下：记录目标位置
  - 鼠标拖动：预览目标朝向
  - 鼠标松开：发布目标到 `/goal_pose`
- 这条交互不会绕过现有调度/行走协议，只是补了一个更顺手的上位机目标输入入口

### 挖掘页实时订阅

- `/digging/debug/segment1_path`
- `/digging/debug/segment2_path`
- `/digging/debug/segment3_path`
- `/digging/debug/optimization_candidate_path`
  - 类型：`nav_msgs/msg/Path`
  - 说明：`trajectory_planner` 发布的三段最终轨迹和优化中间候选轨迹，供挖掘页平面绘图
- `/digging/debug/optimization_metrics`
- `/digging/debug/time_axis`
- `/digging/debug/vgan_result`
- `/digging/debug/vrope_result`
- `/digging/debug/gan_len`
- `/digging/debug/rope_len`
- `/digging/debug/load_rotation_deg`
- `/digging/debug/return_rotation_deg`
  - 类型：`std_msgs/msg/Float32MultiArray`
  - 说明：实时速度、长度、装载/复位回转曲线，以及优化过程指标

`/digging/debug/optimization_metrics` 当前编码：
- `data[0] = eval_count`
- `data[1] = objective_value`
- `data[2] = bucket_fill_rate`
- `data[3] = tf`

QoS 说明：
- 上述 `digging/debug/*` 默认使用 `reliable + transient_local`
- 统一上位机晚于规划节点启动时，仍能接到最近一次调试结果
- `optimization_candidate_path` 是优化过程中的中间候选轨迹，不等于最终采用的第三段轨迹

## 错误码

- `0`：成功
- `1`：busy
- `2`：backend 不可用
- `3`：超时
- `4`：取消
- `5`：legacy/backend 执行失败
