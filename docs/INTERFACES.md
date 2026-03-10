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
- 作用：停止自动调度，并向当前子系统发送 `STOP` 指令。

### `/mission_dispatcher/recover`
- 类型：`std_srvs/srv/Trigger`
- 作用：FAULT 锁存解除后回到 `IDLE`。

## Command 协议

- 大规划对 walk/dig 都只发送两个命令：
  - `START=1`
  - `STOP=2`
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
  - `failed`
- dig 常用 `phase`
  - `idle`
  - `starting`
  - `rotating_scan`
  - `waiting_perception`
  - `resetting`
  - `digging`
  - `completed`
  - `stopping`
  - `stopped`
  - `failed`
- 调度判定规则
  - `completed`：大规划发 `STOP`
  - `stopped` 或 `idle` 且 `active=false`：当前阶段正式收口
  - `error=true`：进入失败处理、重试或 `FAULT`

## 错误码

- `0`：成功
- `1`：busy
- `2`：backend 不可用
- `3`：超时
- `4`：取消
- `5`：legacy/backend 执行失败
