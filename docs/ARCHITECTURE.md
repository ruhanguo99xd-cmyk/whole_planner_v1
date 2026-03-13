# ARCHITECTURE

## 分层

`whole_planner_v1` 采用四层结构：

1. 上位机层：`src/mission_operator_hmi`
   - 统一上位机入口。
   - 提供“大规划 / 行走规划 / 挖掘规划”三个页签切换。
   - 大规划页负责任务提交、开始/停止/恢复。
   - 行走页参考 `autowalk_hmi_qt` 的成熟布局，接入 `map/global_costmap/local_costmap`、目标点、当前位置、物料边界、状态灯、`cmd_vel` 与左右履带速度曲线，并支持地图点选目标。
   - 挖掘页显示 dig 状态、回转角，并直接订阅 `digging/debug/*` 实时画三段轨迹、优化中间候选轨迹和装载/复位曲线。

2. 调度层：`src/mission_dispatcher`
   - 负责任务接收、状态机、walk/dig command action client、超时/重试/故障降级。
3. 规划层：
   - `src/mobility_planner_core`：统一 walk action 服务，支持 `mock` 与 `legacy_autonomous_walk_service`。
   - `src/excavation_planner_core`：统一 dig action 服务，支持 `mock` 与 `legacy_dig_command`。
4. 设备层：`src/plc_adapter`
   - 统一发布 `/plc/status`。
   - 支持 `mock` 回放和 `real` PLC bit 读取。

## 源工程迁入策略

1. 行走核心源码
   - 路径：`src/vendor/mobility_planner_core`
   - 资产：`vendor/mobility_planner_core`
   - 迁入方式：保留原包结构，外侧用 `mobility_planner_core` action 适配。
2. 挖掘核心源码
   - 路径：`src/vendor/excavation_planner_core`
   - 资产：`vendor/excavation_planner_core`
   - 迁入方式：保留原包结构，外侧用 `excavation_planner_core` action 适配。
   - 说明：`tra_planning`、`load`、`return` 已恢复到正式构建链，依赖系统安装的 `libnlopt-dev`。

## 控制流

1. 外部调用 `/mission_dispatcher/submit_mission` 下发任务。
2. 当任务声明 `use_material_target=true` 时，`mission_dispatcher` 先调用 `/mobility/compute_material_target` 解析停靠点。
   - 解析器内部已经拆成三层：`boundary_fit -> work_band -> candidate_evaluation`。
   - 现阶段策略分两类：`edge_near` 和 `far_field`。
   - `boundary_fit` 前面现在还有一层几何输入适配：
     - 直接 request `material_outline`
     - `boundary_input.outline_points`
     - `boundary_input.line_strips`
     - `boundary_input.scatter_points`
     - `/mobility/extract_material_boundary` 在线点云边界提取
   - `scatter_points` 支持从点云投影散点直接进入停靠点规划，在规划层内部先做 ROI 裁剪、角度分桶和边界提取，再进入 `boundary_fit`。
   - 在线点云链路当前为：
     - `PointCloud2`
     - `material_boundary_extractor`
     - `/mobility/extract_material_boundary`
     - `material_target_planner`
     - `boundary_fit -> work_band -> candidate_evaluation`
   - 当前没有 `tf` 树时，点云链路依赖静态外参参数。
     - 如果点云 frame 和目标 frame 不一致但没有配置静态外参，提取器会直接失败。
   - `boundary_fit` 当前已从专利步骤里落实到工程实现：
     - 边界点去重与栅格滤波
     - 边界弧长排序
     - 分段边界曲线采样
     - 边界法向、曲率、拟合质量计算
   - `work_band` 当前已从专利 `S410` 落实到工程实现：
     - 基于边界法向生成 `min/preferred/max` 三层偏置边界
     - 对每个边界站位计算接近距离、朝向变化、障碍净空、局部坡度、估计转弯半径
     - 按 `S414/S415` 裁掉不可达站位
     - 保留最大连通可行站位段
   - `candidate_evaluation` 当前已从专利 `S510` 落实到工程实现：
     - 路径接近代价
     - 参考点代价
     - 偏置距离代价
     - 安全代价（障碍、坡度、转弯半径）
     - 曲率代价
     - 朝向代价
3. `mission_dispatcher` 根据 `/plc/status` 决定进入 `WALK_PREP` 或 `DIG_PREP`。
4. `mission_dispatcher` 发送 `/mobility/execute` 或 `/excavation/execute` 的 `START` 指令。
5. planner core 收到指令后立即回 `received`，真实执行状态通过 `/mobility/status` 或 `/excavation/status` 持续发布。
6. `mission_dispatcher` 观察到 `completed` 后发送 `STOP` 指令收口；外部停机、故障、超时等异常场景发送 `CANCEL` 指令收口。
   - dig cancel 不是只停 orchestrator。
   - `legacy_dig_planner_orchestrator` 会下发 `digging/cancel`。
   - `trajectory_planner` 会对当前 `nlopt` 调用 `force_stop`。
   - `load`、`return` 会在等待服务、等待消息和长计算循环中检查 cancel 并立即放弃当前周期。
7. 再根据状态 topic 驱动 `IDLE -> WALKING -> DIGGING -> IDLE` 或 `FAULT`。

## 数据流

- PLC 输入：`/plc/status` (`integrated_mission_interfaces/msg/PlcSnapshot`)
- 模式输出：`/mission_dispatcher/mode` (`integrated_mission_interfaces/msg/PlannerMode`)
- 行走状态：`/mobility/status` (`integrated_mission_interfaces/msg/SubsystemStatus`)
- 挖掘状态：`/excavation/status` (`integrated_mission_interfaces/msg/SubsystemStatus`)
- 上位机订阅：
  - `/mission_dispatcher/mode`
  - `/mobility/status`
  - `/excavation/status`
  - `/plc/status`
  - `/map`
  - `/global_costmap/costmap`
  - `/local_costmap/costmap`
  - `/cmd_vel`
  - `/plan`
  - `/goal_pose`
  - `/auto_goal_pose`
  - `/mobility/material_boundary`
  - `/lite_slam/swing_angle_deg`
  - `/digging/debug/segment1_path`
  - `/digging/debug/segment2_path`
  - `/digging/debug/segment3_path`
  - `/digging/debug/optimization_candidate_path`
  - `/digging/debug/optimization_metrics`
  - `/digging/debug/time_axis`
  - `/digging/debug/vgan_result`
  - `/digging/debug/vrope_result`
  - `/digging/debug/gan_len`
  - `/digging/debug/rope_len`
  - `/digging/debug/load_rotation_deg`
  - `/digging/debug/return_rotation_deg`
- 上位机调用：
  - `/mission_dispatcher/start`
  - `/mission_dispatcher/stop`
  - `/mission_dispatcher/recover`
  - `/mission_dispatcher/submit_mission`

## 关键设计决策

1. 不把 legacy 业务直接塞进 dispatcher。
   - dispatcher 只做任务编排和故障策略。
2. 保留 legacy 包原名原结构。
   - 方便对比 upstream 和后续增量迁移。
3. 新接口统一放进 `integrated_mission_interfaces`。
   - walk/dig/plc/message 命名不再依赖 legacy 工程各自风格。
4. 先保证最小闭环。
   - mock 可跑。
   - 再逐步切换到 legacy backend。
5. action 只做命令握手，不承载长时执行结果。
   - `START/STOP/CANCEL` 的 result 只表示“收到/拒绝”。
   - 长时执行进度和终态统一走 status topic，避免 legacy action 嵌套带来的重复响应问题。
6. 停靠点规划与调度解耦。
   - `mission_dispatcher` 只负责在 `use_material_target=true` 时调用停靠点服务。
   - 物料边界建模、作业带生成和候选点评价都留在 `mobility_planner_core` 内部，方便后续替换成正式专利算法。
   - 当前已经支持把障碍、坡度和候选点权重从 JSON 约束直接传进规划层，而不需要改 dispatcher。
   - 当前也已经支持把上游边界点、分段线和点云投影散点通过 JSON 约束直接传进规划层，而不需要改 service 定义。
   - 实时 `PointCloud2` 接入单独放在 `material_boundary_extractor` 节点，避免把点云订阅和任务调度耦合进 dispatcher。
   - `material_boundary_extractor` 对 frame mismatch 采取 fail-closed 策略，不会默认假设同坐标系。
7. 上位机不复用 legacy Qt 界面。
   - 现有行走和挖掘工程各自已有独立 Qt HMI，但接口和入口都割裂。
   - 统一上位机直接订阅现有 ROS2 话题和 service，避免把两个老界面硬拼在一起。
   - 行走页保留了 `autowalk_hmi_qt` 最有价值的地图层、状态灯和履带速度可视化体验，并补了地图点选目标交互。
   - 挖掘可视化已经从 CSV 读取升级成实时 topic 订阅，CSV 仅保留为离线存档。
   - `trajectory_planner` 现在会低频发布优化中间候选轨迹和当前指标，HMI 能看到“正在搜索中的候选轨迹”，而不只是最终结果。
