角色关系图（很重要）

           ┌──────────────────┐
           │  挖掘系统 / 行走   │
           │  / 调试工具       │
           └────────┬─────────┘
                    │
         发布指令    │  /machine_mode_cmd
                    ▼
           ┌──────────────────┐
           │  mode_manager    │
           │   （状态机）       │
           └───────┬──────────┘
                   │
        读 PLC     │      写 PLC
                   ▼
              ┌────────┐
              │  PLC   │
              └────────┘
                   │
        广播状态    │  /machine_mode_state
                   ▼
           ┌──────────────────┐
           │  挖掘 / 行走系统   │
           └──────────────────┘

与行走程序的互锁关系图（很重要）
需要满足以下条件才可以行走：
1. machine_mode_state.mode == WALKING
2. walking_active == True
3. mode != PREPARING_TO_WALK
4. mode != STOPPING
5. mode != EMERGENCY_STOP

        ┌──────────────┐
        │ ModeManager  │
        │ (StateMachine│
        │  + PLC)      │
        └─────┬────────┘
              │
   /machine_mode_state
              │
        ┌─────▼────────┐
        │ SafePlcBridge │
        │ (cmd_vel →   │
        │  PLC speed)  │
        └──────────────┘
说明：：：：：：：：-------
📄 一、《行走–挖掘互锁逻辑说明（交付版）》
1. 目的说明

为实现挖掘系统与行走系统的安全串联控制，避免两种作业模式同时激活或误切换，设计并实现一套基于 ROS2 的模式互锁与 PLC 执行控制机制。

该机制适用于如下约束条件：

挖掘 / 行走模式在 PLC 中共用同一使能/激活点位

PLC 不具备复杂模式裁决能力

模式互锁逻辑需在上位（ROS）统一管理

2. 系统角色划分（核心）
2.1 ModeManager（模式管理节点）

职责：

读取 PLC 中当前模式相关点位

判定当前整机所处“逻辑模式”

对外发布统一的 /machine_mode_state 话题

不做的事情：

❌ 不直接控制速度

❌ 不直接参与底层执行

❌ 不绕过互锁直接操作 PLC

2.2 SafePlcBridge（安全执行节点，行走侧）

职责：

接收 /machine_mode_state

根据模式状态进行互锁裁决

仅在“明确允许行走”的条件下，才将 cmd_vel 写入 PLC

安全原则：

默认禁止，一切放行必须被显式满足

3. 统一模式定义（MachineMode）
Mode 编号	模式名称	含义说明
0	IDLE	初始/空闲状态
1	DIGGING	挖掘模式激活
2	WALKING	行走模式激活
3	PREPARING_TO_WALK	行走准备态
4	STOPPING	停止/切换中
5	EMERGENCY_STOP	急停
6	ERROR	故障
4. 互锁核心逻辑（文字版）
4.1 行走允许条件（全部满足才放行）

当前模式 mode == WALKING

PLC 行走点位为 高电平（active = enable）

不处于以下任一状态：

PREPARING_TO_WALK

STOPPING

EMERGENCY_STOP

硬件初始化完成（刹车释放、复位完成）

4.2 行走禁止条件（任一满足即禁止）

未收到模式状态

模式不是 WALKING

任一互锁状态成立

硬件未就绪

禁止行为包括：

停止向 PLC 写速度

强制写入 0 速度

5. 同址点位处理原则（关键说明）

PLC 中的行走 active 点位与 enable 命令为同一物理地址

因此：

不能通过“拉低该点位”来关闭当前模式

只能通过“激活另一模式”来完成模式切换

模式切换逻辑全部由 ModeManager 在 ROS 中完成

PLC 仅作为执行与状态反馈单元。

6. 安全设计原则总结

互锁裁决在 ROS

执行在 PLC

默认禁止

无隐式解锁路径

任意异常 → 行走停止

🧪 二、《互锁测试工况表（PLC + ROS 联调）》

该表可直接用于调试、验收、FAT/SAT

编号	PLC 行走点位	当前 Mode	hardware_ready	期望行为	判定
T1	0	IDLE	❌	不写速度	✔
T2	1	DIGGING	✔	不写速度	✔
T3	1	WALKING	❌	不写速度	✔
T4	1	WALKING	✔	允许写速度	✔
T5	1	PREPARING_TO_WALK	✔	强制停	✔
T6	1	STOPPING	✔	强制停	✔
T7	1	EMERGENCY_STOP	✔	强制停	✔
T8	0	WALKING	✔	强制停	✔
T9	抖动	WALKING	✔	始终安全	✔
T10	1 → 1	DIGGING → WALKING	✔	仅 WALKING 放行	✔

验收重点：

不允许任何“瞬间放行”

不允许在状态未知时行走

不允许通过速度写入反向影响模式

🔁 三、《ModeManager + SafePlcBridge 系统时序图（文字版）》

可直接照此画成 Visio / draw.io

      PLC                ModeManager              SafePlcBridge
       |                      |                         |
       |  行走/挖掘点位状态    |                         |
       |--------------------->|                         |
       |                      |                         |
       |          模式判定逻辑 |                         |
       |                      |                         |
       |                      | 发布 /machine_mode_state|
       |                      |------------------------>|
       |                      |                         |
       |                      |        mode_callback()  |
       |                      |                         |
cmd_vel|                      |                         |
------>|                      |                         |
       |                      |      互锁判断           |
       |                      |  (mode + active + hw)   |
       |                      |                         |
       |                      |      [允许]             |
       |                      |------------------------>|
       |                      |       写速度到 PLC       |
       |                      |                         |
       |                      |      [禁止]             |
       |                      |------------------------>|
       |                      |       写 0 速度          |

一句话总结（你可以原封不动用）

本系统通过 ROS2 统一管理挖掘–行走模式互锁逻辑，
在 PLC 点位复用的前提下，实现了安全、确定、不可绕过的模式切换与执行控制。