# autonomous_walk 现场调用规范（v1）

## 1. 接口定位
- **主链（底层）**：bringup + ekf_bringup + Nav2 + 定位 + 控制
- **autonomous_walk（上层）**：给大规划/调度系统提供统一调用接口

关系：主链负责“能跑”，autonomous_walk负责“好调度”。

## 2. 对外接口
- 服务名：`/autonomous_walk/set_goal`
- 服务类型：`planner_client/srv/SetGoal`

请求核心字段（建议）：
- `goal.header.frame_id: map`
- `goal.pose.position.x/y`
- `goal.pose.orientation`

返回：
- `success: bool`
- `message: string`

## 3. 调用示例
```bash
ros2 service call /autonomous_walk/set_goal planner_client/srv/SetGoal "{
  goal: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 12.0, y: 3.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

## 4. 建议状态机
- `idle`：未初始化
- `ready`：主链与动作服务可用
- `running`：已下发目标，正在行走
- `success`：到达目标
- `fail`：失败（规划失败/控制互锁/超时）
- `stopped`：人工或系统停止

## 5. 失败重试与安全策略
1. 单次失败先重试 1 次（相同目标）
2. 连续失败 >=2 次，触发安全停车（速度置零）
3. 若定位异常（TF断链/odom异常）直接停机并上报
4. PLC互锁触发时禁止继续下发运动目标

## 6. 现场调用流程
1) 启动主链并检查健康
- `/navigate_to_pose` action 可用
- `/cmd_vel_to_plc` 正常
- TF 树完整（map->odom->base_link）

2) 启动 autonomous_walk

3) 大规划调用 `/autonomous_walk/set_goal`

4) 监控过程
- 看 action result
- 看剩余距离反馈

5) 异常处理
- 重试/停机/切人工
