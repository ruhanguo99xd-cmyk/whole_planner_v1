# 外参标定与运动补偿说明

## 场景
设备绕回转中心旋转，QT128 安装在回转中心外且离地有高度。

## 必要参数
- 编码器分辨率：16384 ticks/rev
- 雷达频率：10Hz
- 时间戳：工控机 monotonic time
- 外参：`T_center_lidar = (tx, ty, tz, roll, pitch, yaw)`

## 当前工程实现
1. `deskew.py`
   - 对每个点按 `t_i` 进行角度插值 `theta(t_i)`
   - 反补偿到参考时刻 `ref_ts`
2. `extrinsics.py`
   - 支持 6DoF 外参（含 tz）
3. `calibration.py`
   - 提供一致性优化器（tx/ty/yaw 网格搜索）
   - tz/roll/pitch 可固定输入（后续可扩展联合优化）

## 建议现场流程
1. 先人工测量大概外参填入 `config/default.yaml`
2. 采一圈数据，运行一致性优化微调 tx/ty/yaw
3. 固化外参后再做 map 输出稳定性验证
4. 通过 nav2 使用 `/map` 生成 costmap
