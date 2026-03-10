# 外设接口约定

## QT128

- 输入：SDK 点云流
- 当前：mock 数据
- 目标：统一输出 `LidarFrame(ts, points_xy)`

## PLC 回转编码器

- 地址：`db9999.9999`（按既有程序）
- 当前：mock 角度增量
- 目标：统一输出 `EncoderSample(ts, angle_rad)`

## 时间同步

建议后续加入：
- 同一时钟源（NTP/PTP）
- 采样时间窗匹配
- 插值对齐
