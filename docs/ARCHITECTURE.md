# ARCHITECTURE

## 分层

`whole_planner_v1` 采用三层结构：

1. 调度层：`src/mission_dispatcher`
   - 负责任务接收、状态机、walk/dig command action client、超时/重试/故障降级。
2. 规划层：
   - `src/mobility_planner_core`：统一 walk action 服务，支持 `mock` 与 `legacy_autonomous_walk_service`。
   - `src/excavation_planner_core`：统一 dig action 服务，支持 `mock` 与 `legacy_dig_command`。
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
   - 说明：`tra_planning`、`load`、`return` 已恢复到正式构建链，依赖系统安装的 `libnlopt-dev`。

## 控制流

1. 外部调用 `/mission_dispatcher/submit_mission` 下发任务。
2. `mission_dispatcher` 根据 `/plc/status` 决定进入 `WALK_PREP` 或 `DIG_PREP`。
3. `mission_dispatcher` 发送 `/mobility/execute` 或 `/excavation/execute` 的 `START` 指令。
4. planner core 收到指令后立即回 `received`，真实执行状态通过 `/mobility/status` 或 `/excavation/status` 持续发布。
5. `mission_dispatcher` 观察到 `completed` 后发送 `STOP` 指令收口，再根据状态 topic 驱动 `IDLE -> WALKING -> DIGGING -> IDLE` 或 `FAULT`。

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
5. action 只做命令握手，不承载长时执行结果。
   - `START/STOP` 的 result 只表示“收到/拒绝”。
   - 长时执行进度和终态统一走 status topic，避免 legacy action 嵌套带来的重复响应问题。
