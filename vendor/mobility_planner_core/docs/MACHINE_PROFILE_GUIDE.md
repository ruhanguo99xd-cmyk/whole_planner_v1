# 机型参数管理指南（7机型）

## 为什么这么做
同一套程序在不同机型现场部署时，差异主要是：IP、外参、控制参数、建图参数。
把这些参数收敛到“机型配置文件”，现场只需选择机型，不改代码。

## 目录约定
- `config/machines/M001.yaml` ... `M007.yaml`
- `config/machine.active.yaml`（当前激活机型）
- `scripts/select_machine_profile.py`（切换脚本）

## 现场步骤
1. 选择机型
```bash
python3 scripts/select_machine_profile.py --machine M003
```
2. 核对关键参数
- lidar.host / lidar.port
- encoder.endpoint / db_address
- rtk.host / rtk.port / origin_lat/lon/alt
3. 启动系统（推荐直接在 launch 指定机型）
```bash
ros2 launch bringup ekf_bringup.launch.py machine_profile:=M003 use_lite_slam:=true
```

或先切换 active 文件再启动：
```bash
python3 scripts/select_machine_profile.py --machine M003
ros2 launch bringup ekf_bringup.launch.py use_lite_slam:=true
```

## 常见错误
1) 机型ID拼写错误
- 现象：找不到配置文件
- 处理：确认 `M00X.yaml` 存在

2) IP冲突或不通
- 现象：雷达/PLC连接超时
- 处理：ping设备、核对网段

3) 外参不准
- 现象：地图拉花/重影
- 处理：先回退到上次稳定参数，再做离线标定
