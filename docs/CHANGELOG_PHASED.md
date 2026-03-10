# CHANGELOG_PHASED

## Phase 1
- 新建 `integrated_mission_interfaces`、`mission_dispatcher`、`plc_adapter`、`mobility_planner_core`、`excavation_planner_core`、`mission_bringup`
- 建立 mock walk/dig action
- 建立 mock PLC 回放
- 跑通 `IDLE -> WALKING -> DIGGING -> IDLE`
- 验证：`submit_demo_mission` 触发完整闭环

## Phase 2
- 复制 `anew_autowalk_v3` 到 `src/vendor/mobility_planner_core` 和 `vendor/mobility_planner_core`
- 复制 `tra_planning0226` 到 `src/vendor/excavation_planner_core` 和 `vendor/excavation_planner_core`
- 接入 `autonomous_walk` legacy walk 服务
- 接入 `plc_control_test1` legacy dig command 服务
- 为最小真实链补 `mock_nav2_server`、`legacy_perception_notifier`
- `tra_planning`、`load`、`return` 移除 `COLCON_IGNORE`
- `nlopt` 构建依赖从工作区内置目录切换为系统 `libnlopt-dev`
- walk/dig action 协议改为 `START/STOP` 命令，结果仅返回 `received`
- 执行状态统一改为 `/mobility/status`、`/excavation/status`
- 验证：walk legacy + dig legacy command 跑通一条完整任务链
- dig 主链新增 `legacy_dig_planner_orchestrator`
- `phase2_real` 默认切到 `PRSdata_send + trajectory_planner + load + return` 在线规划链
- 验证：`trajectory_planner/load/return` 已通过 dig action 在线跑通，不再只是编译通过

## Phase 3
- PLC real bit 映射改为参数化
- dispatcher 增加 `recover` 服务
- dispatcher 在 FAULT 转移时主动取消当前 goal
- 增加 PLC 映射单测
- 修复 legacy stop 收口顺序，消除重复 action 响应告警
- 验证：9 项单测通过，真实链回归通过

## Phase 4
- 新增架构文档、接口文档、Runbook、阶段变更记录
- 补充 Phase 2 构建/启动脚本
- 固化当前风险与后续解锁策略
