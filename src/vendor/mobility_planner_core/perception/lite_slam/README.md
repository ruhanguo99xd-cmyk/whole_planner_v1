# lite_slam（QT128 + PLC回转编码器）

这份 README 是按你当前真实需求重写的：

- 激光雷达安装在**电铲回转中心外**，且有**离地高度**；
- 通过设备回转一圈采集数据；
- 编码器数据来自 PLC（S7，地址 `db9999.9999`）；
- 编码器硬件测量频率 1000Hz；
- 需要点级运动补偿 + 外参标定（含高度）。

---

## 1. 我理解的需求（先对齐）

你的任务不是常规“车辆边走边建图”的 SLAM，而是：

1) 设备围绕回转中心旋转；
2) QT128 连续出点云（10Hz）；
3) PLC 给出高频回转编码器角度（1000Hz）；
4) 通过编码器角度对每个激光点做时间补偿（deskew）；
5) 因为雷达不在回转中心，必须做 `T_center_lidar` 外参标定（含 `tz` 高度项）；
6) 最终输出 `/map`（OccupancyGrid）作为导航上游。

如果你认可上面 6 条，就说明需求理解一致。

---

## 2. 为什么先输出 `/map`，而不是直接 costmap

- `/map` 是建图层的标准输出，稳定、通用；
- `global_costmap/local_costmap` 属于导航层（Nav2）动态生成；
- 这样和 `anew_autowalk` 的耦合最小，后续替换/升级更方便。

---

## 3. 当前实现到什么程度

### 已完成
- 编码器分辨率配置：`16384 ticks/rev`
- 编码器程序采样频率可配置，默认已设 `1000Hz`
- 时间基准统一为工控机 `monotonic` 时钟
- 点级运动补偿（每点按 `t_i` 插值角度）
- 外参模型支持 6DoF（`tx,ty,tz,roll,pitch,yaw`）
- 2D 栅格融合输出（OccupancyGrid内部表示）
- `ekf_bringup.launch.py` 一键开关：`use_lite_slam:=true`
- 离线标定工具 v1

### 仍是 v1（后续可增强）
- 离线标定 v1 当前优化 `tx/ty/tz/yaw`，`roll/pitch` 先固定
- ROS2 `/map` 正式发布器仍在适配层占位（核心算法已就位）

---

## 4. 核心原理（简版）

### 4.1 点级运动补偿（deskew）
对每个激光点：
- 点时刻：`t_i = frame_ts + rel_ts_i`
- 编码器插值角度：`theta(t_i)`
- 参考时刻角度：`theta(ref)`（通常取帧尾）
- 旋转差：`dtheta = theta(t_i) - theta(ref)`
- 将该点反旋转 `-dtheta` 到统一时刻

这样可以消除“扫描期间设备在转”导致的扭曲。

### 4.2 外参补偿（含高度）
雷达点先从 lidar 坐标系变到回转中心坐标系：

`p_center = R(roll,pitch,yaw) * p_lidar + [tx,ty,tz]`

其中 `tz` 就是你关心的安装高度偏置。

### 4.3 栅格建图
把补偿后的点投影到 XY 平面，做 log-odds 占据更新，得到 `/map` 基础图层。

---

## 5. 编码器 1000Hz：硬件频率 vs 程序频率

你提的问题完全正确：

- **硬件测量频率（PLC端）**：1000Hz
- **程序读取频率（软件端）**：由程序决定

建议策略：
- 程序读取尽量跟硬件一致（默认 1000Hz），避免信息先天丢失；
- 算法内部按需要重采样/插值，不要在采集前粗暴降采样。

配置位置：`config/default.yaml`

```yaml
lidar:
  host: 192.168.1.201   # 现场改成QT128真实IP
  port: 2368            # 现场改成真实端口

encoder:
  protocol: s7
  endpoint: 192.168.1.10   # 现场改成PLC真实IP
  db_address: "db9999.9999"
  resolution_ticks_per_rev: 16384
  sample_hz: 1000.0
```

---

## 6. 工程结构

```text
lite_slam/
  config/default.yaml
  docs/
    CALIBRATION.md
    OFFLINE_CALIBRATION_V1.md
  src/lite_slam/
    app.py
    interfaces/
      hesai_qt128.py
      plc_encoder.py
    slam/
      deskew.py
      extrinsics.py
      calibration.py
      mapper.py
      occupancy_grid.py
    tools/
      record_dataset_v1.py
      offline_calibrate_v1.py
```

---

## 7. 实操步骤（建议现场按这个流程）

### Step A：先录一圈数据
```bash
cd /home/ruhanguo/anew_autowalk_v3/src/perception/lite_slam
PYTHONPATH=src python3 -m lite_slam.tools.record_dataset_v1 \
  --config config/default.yaml \
  --seconds 30 \
  --out_dir data_samples/run1
```

输出：
- `data_samples/run1/lidar_points.csv`
- `data_samples/run1/encoder.csv`

### Step B：跑离线外参标定 v1
```bash
PYTHONPATH=src python3 -m lite_slam.tools.offline_calibrate_v1 \
  --lidar_csv data_samples/run1/lidar_points.csv \
  --encoder_csv data_samples/run1/encoder.csv \
  --init_tx 0.8 --init_ty -0.25 --init_tz 2.1 --init_yaw 0.0 \
  --z_target 0.0
```

把输出最优外参写回 `config/default.yaml`。

### Step C：集成运行
```bash
ros2 launch bringup ekf_bringup.launch.py use_lite_slam:=true
```

---

## 8. 你最关心的风险点（我直接写明）

1) 时间戳不同源（PLC时钟 vs 工控机时钟）会引入补偿误差。
   - 当前实现按工控机 monotonic 对齐。若 PLC 时间不可直取，必须做通信延迟校正。

2) 外参初值太差会影响 v1 网格搜索收敛速度。
   - 建议先卷尺测一个粗初值（尤其 `tz`）。

3) 10Hz 雷达 + 低速回转可以，若转速过快会加重扭曲。
   - 需控制回转速度，保证每圈点密度充足。

---

## 9. 下一步（我建议）

- 做 ROS2 原生 `/map` 发布与 `.pgm/.yaml` 保存；
- 离线标定 v2：接 Ceres，联合优化 `roll/pitch`；
- 增加“标定质量报告”（重投影误差、扇区一致性、地面残差）。

---

如果你愿意，我下一步直接补：
**“采集→标定→回写配置→启动验证” 一键脚本**，你现场只用一条命令跑全流程。