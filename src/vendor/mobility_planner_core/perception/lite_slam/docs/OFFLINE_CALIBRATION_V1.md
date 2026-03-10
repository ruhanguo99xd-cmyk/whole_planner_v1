# 离线标定工具 v1（offline_calibrate_v1）

## 1. 原理

目标：估计 `T_center_lidar = (tx, ty, tz, roll, pitch, yaw)`，其中 **tz 是雷达到回转中心坐标系的高度项**。

对每个候选外参，流程如下：

1. 对每帧点云做点级运动补偿（deskew）
   - 每个点按 `t_i` 插值得到编码器角度 `theta(t_i)`
   - 反补偿到统一参考时刻
2. 用候选外参把点从 lidar 坐标变换到回转中心坐标
3. 计算联合评分：
   - **地图锐度项**：补偿后 2D 栅格熵越低越好（边界更清晰）
   - **地面高度项**：最低 10% 点的均值高度越接近 `z_target` 越好
4. 网格搜索 `tx/ty/tz/yaw`（v1），`roll/pitch` 先固定

> v1 是工程可用版：稳、可解释、易调。后续可接 Ceres 做连续优化与不确定性估计。

## 2. 输入数据格式

### lidar_csv
列头必须包含：
- `frame_id,frame_ts,rel_ts,x,y,z`

### encoder_csv
列头必须包含：
- `ts,angle_rad`
- 可选：`ticks`

## 3. 运行示例

```bash
PYTHONPATH=src python3 -m lite_slam.tools.offline_calibrate_v1 \
  --lidar_csv data/lidar_points.csv \
  --encoder_csv data/encoder.csv \
  --init_tx 0.8 --init_ty -0.25 --init_tz 2.1 --init_yaw 0.0 \
  --z_target 0.0
```

输出：最优外参与评分。

## 4. 关于编码器 1000Hz 与程序采样频率

你的理解是对的：
- **硬件测量频率**（PLC端）= 1000Hz
- **程序读取频率**（软件端）= 由程序参数决定

建议：
- 如果工控机性能允许，先把程序读取频率设到 `1000Hz`，尽量不丢信息；
- 再在算法内部按需要做重采样/插值（而不是前面就降采样）。

当前 `lite_slam` 配置里可直接设：
- `encoder.sample_hz: 1000.0`

这样点级 deskew 的角度插值精度会更好。
