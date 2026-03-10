# INTERFACES

## 调度服务

### `/mission_dispatcher/submit_mission`
- 类型：`integrated_mission_interfaces/srv/SubmitMission`
- 输入：
  - `mission_id`
  - `walk_target`
  - `dig_target_zone`
  - `walk_constraints_json`
  - `dig_parameters_json`
  - `priority`
- 输出：
  - `accepted`
  - `message`

### `/mission_dispatcher/start`
- 类型：`std_srvs/srv/Trigger`
- 作用：启用自动调度。

### `/mission_dispatcher/stop`
- 类型：`std_srvs/srv/Trigger`
- 作用：停止自动调度并取消当前 action。

### `/mission_dispatcher/recover`
- 类型：`std_srvs/srv/Trigger`
- 作用：FAULT 锁存解除后回到 `IDLE`。

## Walk Action

### 名称
- `/mobility/execute`
- 类型：`integrated_mission_interfaces/action/WalkMission`

### Goal
- `mission_id`
- `target_pose`
- `constraints_json`
- `priority`
- `timeout_sec`

### Result
- `success`
- `error_code`
- `message`
- `retryable`

### Feedback
- `phase`
- `progress`

## Dig Action

### 名称
- `/excavation/execute`
- 类型：`integrated_mission_interfaces/action/DigMission`

### Goal
- `mission_id`
- `target_zone`
- `process_parameters_json`
- `safety_boundary_json`
- `priority`
- `timeout_sec`

### Result
- `success`
- `error_code`
- `message`
- `retryable`
- `material_volume`

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
  - `active`
  - `ready`
  - `error`
  - `error_code`
  - `phase`
  - `detail`

## 错误码

- `0`：成功
- `2`：backend 不可用
- `3`：超时
- `4`：取消
- `5`：legacy/backend 执行失败
