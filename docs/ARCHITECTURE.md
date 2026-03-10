# ARCHITECTURE

## 分层

`whole_planner_v1` 采用三层结构：

1. 调度层：`src/mission_dispatcher`
   - 负责任务接收、状态机、walk/dig action client、超时/重试/故障降级。
2. 规划层：
   - `src/mobility_planner_core`：统一 walk action 服务，支持 `mock` 与 `legacy_autonomous_walk_service`。
   - `src/excavation_planner_core`：统一 dig action 服务，支持 `mock` 与 `legacy_dig_action`。
3. 设备层：`src/plc_adapter`
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
   - 说明：`tra_planning`、`load`、`return` 因当前环境缺少 `nlopt`，默认带 `COLCON_IGNORE` 保持源码保留、构建关闭。

## 控制流

1. 外部调用 `/mission_dispatcher/submit_mission` 下发任务。
2. `mission_dispatcher` 根据 `/plc/status` 决定进入 `WALK_PREP` 或 `DIG_PREP`。
3. `mission_dispatcher` 调用 `/mobility/execute` 或 `/excavation/execute` action。
4. `mobility_planner_core` / `excavation_planner_core` 根据 backend 切到 mock 或 legacy 实现。
5. 执行完成后将结果回传 `mission_dispatcher`，驱动 `IDLE -> WALKING -> DIGGING -> IDLE` 或 `FAULT`。

## 数据流

- PLC 输入：`/plc/status` (`integrated_mission_interfaces/msg/PlcSnapshot`)
- 模式输出：`/mission_dispatcher/mode` (`integrated_mission_interfaces/msg/PlannerMode`)
- 行走状态：`/mobility/status` (`integrated_mission_interfaces/msg/SubsystemStatus`)
- 挖掘状态：`/excavation/status` (`integrated_mission_interfaces/msg/SubsystemStatus`)

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
