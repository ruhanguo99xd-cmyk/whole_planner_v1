# whole_planner_v1

正式工程名：`integrated_mission_planner`

本仓库用于融合行走规划 `anew_autowalk_v3` 与挖掘规划 `tra_planning0226`，并在上层提供统一调度、动作切换与 PLC 适配。

## 目录

- `src/`：ROS 2 包
- `src/vendor/`：复制进来的 legacy 源码
- `vendor/`：legacy 配置、文档、脚本和数据资产
- `docs/ARCHITECTURE.md`：架构说明
- `docs/INTERFACES.md`：接口冻结说明
- `docs/RUNBOOK.md`：构建与启动手册
- `docs/CHANGELOG_PHASED.md`：阶段交付记录
