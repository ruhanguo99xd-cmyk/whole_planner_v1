# lite_slam 架构设计（v0.1）

## 1. 目标

构建可独立运行、可并入 `anew_autowalk` 的 2D 栅格 SLAM 子工程。

## 2. 分层

1. **Interface 层**
   - `HesaiQT128Source`
   - `PLCEncoderSource`
2. **Core SLAM 层**
   - 编码器增量位姿更新
   - 点云转平面 + 栅格积分
3. **Adapter 层**
   - ROS2 发布 `/map`
   - 地图保存 / 上层 RPC

## 3. 与 anew_autowalk 的兼容点

- 输出使用标准 OccupancyGrid 语义
- 配置集中在 YAML，便于被上层统一加载
- 接口独立，后续可切换真实驱动/录包回放

## 4. 输出建议

- **短期**：只产出 `/map` + 地图文件
- **中期**：由导航层生成 `global_costmap`、`local_costmap`
